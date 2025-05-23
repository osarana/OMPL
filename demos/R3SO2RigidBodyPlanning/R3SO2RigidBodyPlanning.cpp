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
#include <iostream>
#include <chrono>
#include <ompl/base/samplers/UniformValidStateSampler.h>

using namespace ompl;

bool printIntPath(const geometric::PathGeometric &path, const base::SpaceInformationPtr &si, const std::string &filename) {
	geometric::PathGeometric geometricPath(path);

	geometricPath.interpolate(1000);

	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file: " << filename << std::endl;
		return false;
	}

	// Print the interpolated path in matrix form
    file << "Interpolated Solution Path:\n";

    bool valid = true;
    for (size_t i = 0; i < geometricPath.getStateCount(); ++i)
    {
        const base::State *state = geometricPath.getState(i);

		if (!si->isValid(state)) {
			std::cout << "INVALID STATE COMPUTED AT INDEX: " << i << std::endl;
			valid = false;
			continue;
		}
        
        const auto *compoundState = state->as<base::CompoundState>();
        const auto *r3State = compoundState->as<base::RealVectorStateSpace::StateType>(0);
        const auto *so2State = compoundState->as<base::SO2StateSpace::StateType>(1);

        // Print the state in "x y z yaw" format
        file << r3State->values[0] << " " << r3State->values[1] << " " << r3State->values[2] << " " << so2State->value << "\n";
    }

    if (valid) {
    	std::cout << "Interpolated path is valid.\n";
    } else {
    	std::cerr << "Interpolated path contains invalid states.\n";
    }

    file.close();
    return valid;
}

bool validCheck(const geometric::PathGeometric &path, const base::SpaceInformationPtr &si) {
	for (size_t i = 0; i < path.getStateCount(); ++i) {
		if (!si->isValid(path.getState(i))) {
			std::cerr << "INVALID STATE FOUND IN PATH\n";
			return false;
		}
	}

	return true;
}

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

    // define upper and lower bounds
    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, std::stod(configSettings["volume.max.x"]));
    bounds.setHigh(1, std::stod(configSettings["volume.max.y"]));
    bounds.setHigh(2, std::stod(configSettings["volume.max.z"]));
    bounds.setLow(0,  std::stod(configSettings["volume.min.x"]));
    bounds.setLow(1,  std::stod(configSettings["volume.min.y"]));
    bounds.setLow(2,  std::stod(configSettings["volume.min.z"]));
    setup.getStateSpace()->as<base::R3SO2StateSpace>()->setBounds(bounds);

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

	setup.setPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));
    // setting collision checking resolution to 1% of the space extent
    double resolution = 0.000888888; // thickness of the robot's arm
    setup.getSpaceInformation()->setStateValidityCheckingResolution(resolution);

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

    bool solutionFound = false;
    
	for (auto &planner : planners) {
		setup.setPlanner(planner);
		resolution = 0.000888888;

		auto start_time = std::chrono::high_resolution_clock::now();

		solutionFound = false;
		while (!solutionFound && resolution > 1e-8) {
			setup.getSpaceInformation()->setStateValidityChecker(
				[](const base::State *) { return true; });
				
			setup.getSpaceInformation()->setStateValidityCheckingResolution(resolution);

			setup.getSpaceInformation()->setValidStateSamplerAllocator(
				[](const base::SpaceInformation *si) {
					return std::make_shared<base::UniformValidStateSampler>(si);	
				});
		
			setup.getPlanner()->clear();
			setup.getProblemDefinition()->clearSolutionPaths();
		    setup.getProblemDefinition()->clearGoal();
		    setup.getProblemDefinition()->clearStartStates();
		    setup.setStartAndGoalStates(start, goal);
		    
			setup.setup();
			std::cout << "Resolution: " << resolution << std::endl;

			if (setup.solve(3600)) {
				std::cout << "Original path: " << std::endl;
				setup.getSolutionPath().printAsMatrix(std::cout);
				geometric::PathGeometric path = setup.getSolutionPath();

				if (validCheck(path, setup.getSpaceInformation())) {
					std::cout << "Original path is valid.\n";
					if (printIntPath(path, setup.getSpaceInformation(), "TESTPATH.txt")) {
						std::cout << "Final interpolated path is valid.\n";
						solutionFound = true;
						break;
					} else {
						std::cerr << "Invalid interpolated path.\n";
						resolution /= 2;
					}
				} else {
					std::cerr << "Invalid original path.\n";
					resolution /= 2;
				}
			} else {
				std::cerr << "No solution found with this planner.\n";
				resolution /= 2;
			}
		}

		auto end_time = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> duration = end_time - start_time;
		std::cout << "Planner runtime: " << duration.count() << " seconds.\n";

	}

	if (!solutionFound) {
		std::cerr << "Failed to find a valid solution with any planners.\n";
	}

    return 0;
}
