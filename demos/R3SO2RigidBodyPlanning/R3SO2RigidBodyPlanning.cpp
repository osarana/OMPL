/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Joe Masterjohn */

#include <omplapp/apps/R3SO2RigidBodyPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

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

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(1e-15);

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

    // try to solve the problem
    if (setup.solve(1e6))
    {
        // simplify & print the solution
		// setup.simplifySolution();
        setup.getSolutionPath().printAsMatrix(std::cout);
    }

    return 0;
}
