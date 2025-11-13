/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Joe Masterjohn/Oscar Arana */

#include <omplapp/apps/R3SO2RigidBodyPlanning.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <omplapp/config.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

#include <iostream>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include <tinyxml2.h>
#include <iomanip>
#include <cctype>

using namespace ompl;

static double inferArmLenFromConfigFilename(const std::string &cfgPath)
{
    // basename (strip directory)
    std::size_t slash = cfgPath.find_last_of("/\\");
    std::string fname = (slash == std::string::npos) ? cfgPath : cfgPath.substr(slash + 1);

    // stem (strip extension)
    std::size_t dot = fname.find_last_of('.');
    std::string stem = (dot == std::string::npos) ? fname : fname.substr(0, dot); // e.g., "vtk_1_15"

    // get trailing digits/underscores, e.g., "1_15"
    int i = static_cast<int>(stem.size()) - 1;
    while (i >= 0 && (std::isdigit(static_cast<unsigned char>(stem[static_cast<size_t>(i)])) || stem[static_cast<size_t>(i)] == '_'))
        --i;
    std::string num = stem.substr(static_cast<size_t>(i + 1));
    if (num.empty()) return 1.0; // fallback

    // Treat first '_' as decimal point, others as fractional digits
    std::size_t pos = num.find('_');
    if (pos == std::string::npos) {
        try { return std::stod(num); } catch (...) { return 1.0; }
    } else {
        std::string intPart  = num.substr(0, pos);
        std::string fracPart = num.substr(pos + 1);
        // remove any remaining underscores in frac
        fracPart.erase(std::remove(fracPart.begin(), fracPart.end(), '_'), fracPart.end());
        if (intPart.empty()) intPart = "0";
        try { return std::stod(intPart + "." + fracPart); } catch (...) { return 1.0; }
    }
}

static bool pathEdgesAreValid (const geometric::PathGeometric &path, const base::SpaceInformationPtr &si, std::size_t &badIndex) {
	if (path.getStateCount() < 2) return true;

	for (std::size_t i = 0; i + 1 < path.getStateCount (); ++i)
	{
		if (!si->checkMotion (path.getState (i), path.getState (i + 1))) {
			badIndex = i;
			return false;
		}
	}

	return true;
}

static bool pathStatesAreValid (const geometric::PathGeometric &path, const base::SpaceInformationPtr &si, std::size_t &badIndex) {
	for (std::size_t i = 0; i < path.getStateCount (); ++i) 
	{
		if (!si->isValid (path.getState (i))) {
			badIndex = i;
			return false;
		}
	}

	return true;
}

static void writeStateLine (std::ofstream &file, const base::SpaceInformationPtr &si, const base::State *s, bool &allValid, const base::StateValidityCheckerPtr &svc) {
	if (svc && !svc->isValid (s)) allValid = false;

	const auto *compound = s->as<base::CompoundState>();
    const auto *r3  = compound->as<base::RealVectorStateSpace::StateType>(0);
    const auto *so2 = compound->as<base::SO2StateSpace::StateType>(1);
    file << r3->values[0] << " " << r3->values[1] << " "
         << r3->values[2] << " " << so2->value << "\n";
}

static void streamSegment (std::ofstream &file, const base::SpaceInformationPtr &si, const base::State *a, const base::State *b, double maxSegLen, bool &allValid, const base::StateValidityCheckerPtr &svc) {
	double d = si->distance (a, b);
	if (d <= maxSegLen || d == 0.0) {
		writeStateLine (file, si, a, allValid, svc);
		return;
	}

	base::State *mid = si->allocState ();
	si->getStateSpace()->interpolate (a, b, 0.5, mid);
	streamSegment(file, si, a, mid, maxSegLen, allValid, svc);
	streamSegment(file, si, mid, b, maxSegLen, allValid, svc);
	si->freeState(mid);
}

static bool streamPathToFile (const geometric::PathGeometric &inPath, const base::SpaceInformationPtr &si, const std::string &filename) {
	if (inPath.getStateCount () == 0) return false;

	std::ofstream file (filename);
	if (!file.is_open ()) {
		std::cerr << "Failed to open file: " << filename << "\n";
		return false;
	}

	file << "Interpolated Solution Path:\n";
	bool allValid = true;

	const double segLen = si->getStateSpace()->getLongestValidSegmentLength();
	auto svc = si->getStateValidityChecker();

	for (std::size_t i = 0; i + 1 < inPath.getStateCount(); ++i) 
	{
		const base::State *a = inPath.getState (i);
		const base::State *b = inPath.getState (i + 1);
		streamSegment (file, si, a, b, segLen, allValid, svc);
		if (i + 1 == inPath.getStateCount () - 1) {
			writeStateLine (file, si, b, allValid, svc);
		}
	}

	file.close ();
	return allValid;
}

/*
static void interpolateToCurrentResolution (geometric::PathGeometric &path, const base::SpaceInformationPtr &si) {
	const double segLen = si->getStateSpace()->getLongestValidSegmentLength();
	const double total = path.length ();
	unsigned N = (segLen > 0.0) ? static_cast<unsigned>(std::ceil (total / segLen)) : 1000u;
	N = std::max<unsigned> (N, 2u);
	path.interpolate (N);
}
*/

/*
bool printIntPath(const geometric::PathGeometric &inPath, const base::SpaceInformationPtr &si, const std::string &filename) {
	geometric::PathGeometric path(inPath);
	interpolateToCurrentResolution (path, si);
	
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filename << std::endl;
		return false;
	}

	// Print the interpolated path in matrix form
    file << "Interpolated Solution Path:\n";
    bool valid = true;
    
    for (size_t i = 0; i < path.getStateCount(); ++i)
    {
        const base::State *state = path.getState(i);

		if (!si->isValid(state)) {
			std::cout << "INVALID STATE COMPUTED AT INDEX: " << i << std::endl;
			valid = false;
		}
        
        const auto *compoundState = state->as<base::CompoundState>();
        const auto *r3State = compoundState->as<base::RealVectorStateSpace::StateType>(0);
        const auto *so2State = compoundState->as<base::SO2StateSpace::StateType>(1);

        // Print the state in "x y z yaw" format
        file << r3State->values[0] << " " << r3State->values[1] << " " << r3State->values[2] << " " << so2State->value << "\n";
    }

    file.close();
    return valid;
}
*/

static void retunePlannerRange (const base::SpaceInformationPtr &si, const std::shared_ptr<base::Planner> &planner) {
	const double maxExtent = si->getMaximumExtent ();
	const double segLen = si->getStateSpace()->getLongestValidSegmentLength();

	double range = std::min (10.0 * segLen, 0.05 * maxExtent);
	range = std::max (range, 1e-12);

	if (auto p = std::dynamic_pointer_cast<geometric::RRTConnect>(planner)) p->setRange (range);
	if (auto p = std::dynamic_pointer_cast<geometric::RRT>(planner)) p->setRange (range);
	if (auto p = std::dynamic_pointer_cast<geometric::KPIECE1>(planner)) p->setRange (range);
	if (auto p = std::dynamic_pointer_cast<geometric::BKPIECE1>(planner)) p->setRange (range);
	if (auto p = std::dynamic_pointer_cast<geometric::LBKPIECE1>(planner)) p->setRange (range);
	if (auto p = std::dynamic_pointer_cast<geometric::SBL>(planner)) p->setRange (range);
	if (auto p = std::dynamic_pointer_cast<geometric::EST>(planner)) p->setRange (range);
}

struct ReportRow {
	std::string plannerName;
	double armLen;
	double stepSizeInitial;
	int	   numHalvings;
	double timeout;
	double finalStep;
	double timeSec;
	bool   solved;
};

int main(int argc, char **argv)
{
	if (argc != 2)
	{
		std::cerr << "Usage: " << argv[0] << " <config.file.txt>" << std::endl;
		return -1;
	}

	// Open the configuration file
	std::ifstream configFile(argv[1]);
	if (!configFile)
	{
		std::cerr << "Error: Could not open the configuration file." << std::endl;
		return -2;		
	}

	// Create a map to store the configuration settings
	std::map<std::string, std::string> configSettings;

	// Read and parse the config file
	std::string line;
	while(std::getline(configFile, line))
	{
		std::istringstream iss(line);
		std::string key, value;
		if (std::getline(iss, key, '=') && std::getline(iss, value))
		{
			// Trim any whitespaces
			key.erase(0, key.find_first_not_of(" \t"));
			key.erase(key.find_last_not_of(" \t") + 1);
			value.erase(0, value.find_first_not_of(" \t"));
			value.erase(value.find_last_not_of(" \t") + 1);
			configSettings[key] = value;			
		}		
	}
	
    // plan in R3SO2
    app::R3SO2RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = configSettings["robot"];
    std::string env_fname = configSettings["world"];
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

	// define upper and lower bounds
    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, std::stod(configSettings["volume.max.x"]));
    bounds.setHigh(1, std::stod(configSettings["volume.max.y"]));
    bounds.setHigh(2, std::stod(configSettings["volume.max.z"]));
    bounds.setLow(0,  std::stod(configSettings["volume.min.x"]));
    bounds.setLow(1,  std::stod(configSettings["volume.min.y"]));
    bounds.setLow(2,  std::stod(configSettings["volume.min.z"]));
    setup.getStateSpace()->as<base::R3SO2StateSpace>()->setBounds(bounds);

    // define start state
    base::ScopedState<base::R3SO2StateSpace> start(setup.getSpaceInformation());
    start->setX(std::stod(configSettings["start.x"]));
    start->setY(std::stod(configSettings["start.y"]));
    start->setZ(std::stod(configSettings["start.z"]));
    start->setYaw(std::stod(configSettings["start.theta"]));

    // define goal state
    base::ScopedState<base::R3SO2StateSpace> goal(start);
    goal->setX(std::stod(configSettings["goal.x"]));
    goal->setY(std::stod(configSettings["goal.y"]));
    goal->setZ(std::stod(configSettings["goal.z"]));
    goal->setYaw(std::stod(configSettings["goal.theta"]));

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // List of planners
    std::vector<std::shared_ptr<base::Planner>> planners = {
        std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation()),
        std::make_shared<geometric::RRT>(setup.getSpaceInformation()),
        std::make_shared<geometric::BKPIECE1>(setup.getSpaceInformation()),
        std::make_shared<geometric::LBKPIECE1>(setup.getSpaceInformation()),
        std::make_shared<geometric::KPIECE1>(setup.getSpaceInformation()),
       	std::make_shared<geometric::SBL>(setup.getSpaceInformation()),
        std::make_shared<geometric::EST>(setup.getSpaceInformation()),
       	std::make_shared<geometric::PRM>(setup.getSpaceInformation())
    };

	
    // setting collision checking resolution to 1% of the space extent
    double resolution = 0.000888888; // thickness of the robot's arm
	double armLen = inferArmLenFromConfigFilename (argv[1]);
    
	auto si = setup.getSpaceInformation ();
	si->setStateValidityCheckingResolution (resolution);
	si->getStateSpace ()->setValidSegmentCountFactor (5.0);
	si->setValidStateSamplerAllocator ([](const base::SpaceInformation *insi) {
		return std::make_shared<base::UniformValidStateSampler>(insi);
	});

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

	std::vector<ReportRow> report;
    
	bool solutionFound = false;
	for (auto &planner : planners) {
		setup.setPlanner(planner);
		
		resolution = 0.000888888;
		int halvings = 0;
		bool solvedThisPlanner = false;
		auto start_time = std::chrono::high_resolution_clock::now();

		while (!solvedThisPlanner && resolution > 1e-8) {
			double extent = si->getMaximumExtent ();
			double frac = (extent > 0) ? resolution / extent : 0.00088888;
			si->getStateSpace()->setLongestValidSegmentFraction (frac);
			retunePlannerRange (si, planner);
		
			setup.getPlanner()->clear();
			setup.getProblemDefinition()->clearSolutionPaths();
		    setup.getProblemDefinition()->clearGoal();
		    setup.getProblemDefinition()->clearStartStates();
		    setup.setStartAndGoalStates(start, goal);
			setup.getPlanner()->setup();
			
			const double segLen = si->getStateSpace()->getLongestValidSegmentLength();
			std::cout << "Planner: " << setup.getPlanner()->getName()
					  << " | resolution = " << resolution
					  << " | segLen = " << segLen
					  << " | solve(s) = 3600" << std::endl;

			if (setup.solve(1e8)) {
				geometric::PathGeometric path = setup.getSolutionPath ();
				std::cout << "Raw path (matrix):\n";
				path.printAsMatrix (std::cout);

				std::size_t badIdx = 0;
				if (!pathStatesAreValid (path, si, badIdx)) {
					std::cerr << "[FAIL] Invalid state at " << badIdx
							  << " - halving resolution\n";
					resolution *= 0.5; ++halvings;
					continue;
				}

				if (!pathEdgesAreValid (path, si, badIdx)) {
					std::cerr << "[FAIL] Invalid motion between "
							  << badIdx << " and " << badIdx + 1
							  << " - halving resolution\n";
					resolution *= 0.5; ++halvings;
					continue; 
				}

				if (!streamPathToFile (path, si, "TESTPATH.txt")) {
					std::cerr << "[FAIL] Interpolated path invalid - halving resolution\n";
					resolution *= 0.5; ++halvings;
					continue;
				}

				std::cout << "[OK] Path valid at resolution = " << resolution << " (file: TESTPATH.txt)\n";

				solutionFound = true;
				solvedThisPlanner = true;
			} else {
				std::cerr << "No solution at resolution = " << resolution << " - halving.\n";
				resolution *= 0.5; ++halvings;
			}
		}

		auto end_time = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> duration = end_time - start_time;
		
		std::cout << "Planner " << setup.getPlanner()->getName()
              << " runtime: " << duration.count() << " seconds. "
              << (solvedThisPlanner ? "SOLVED" : "FAILED")
              << " (final resolution=" << resolution << ")\n";

		report.push_back (ReportRow{
            setup.getPlanner()->getName(),   // Planner
            armLen,                          // Arm Len (from config)
            8e-3,                            // Step Size (initial)
            halvings,                        // # of Subdivisions (halvings)
            3600,		                     // Timeout (per attempt)
			resolution,                      // Final Step (final res)
            duration.count(),                // Time (seconds)
            solvedThisPlanner                // solved?
        });
	}

	std::cout << "\n=== SUMMARY REPORT ===\n";
	std::cout << "Planner, Arm Len, Step Size, # of Subdivisions, Timeout, Final Step, Time Solved\n";
	std::cout << std::setprecision (10);

	for (const auto &r : report)
	{
		std::cout << r.plannerName << ","
                  << r.armLen << ","
                  << r.stepSizeInitial << ","
                  << r.numHalvings << ","
                  << r.timeout << ","
                  << r.finalStep << ","
                  << r.timeSec << ","
                  << (r.solved ? "1" : "0") << "\n";
	}

	const char *csvName = "REPORT.csv";
	bool writeHeader = false;
	{
		std::ifstream test(csvName);
		if (!test.good ()) writeHeader=true;
	}
	std::ofstream csv (csvName, std::ios::app);
	if (!csv.is_open ()) {
		std::cerr << "Failed to open " << csvName << " for writing.\n ";
	} else {
		if (writeHeader)
			csv << "Planner,Arm Len,Step Size,# of Subdivisions,Timeout,Final Step,Time,Solved\n";
        
		csv << std::setprecision(10);
        for (const auto &r : report)
        {
            csv << r.plannerName << ","
                << r.armLen << ","
                << r.stepSizeInitial << ","
                << r.numHalvings << ","
                << r.timeout << ","
                << r.finalStep << ","
                << r.timeSec << ","
                << (r.solved ? "1" : "0") << "\n";
        }
        csv.close();
        std::cout << "Wrote report rows to " << csvName << "\n";
	}

	bool anySolved = false;
    for (const auto &r : report) if (r.solved) { anySolved = true; break; }
    if (!anySolved)
    {
        std::cerr << "Failed to find a valid solution with any planner down to res=" << 1e-8 << "\n";
        return 1;
    }


    return 0;
}
