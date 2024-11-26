#include <omplapp/apps/R3SO2RigidBodyPlanning.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <omplapp/config.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using namespace ompl;

void checkPathValidityFromFile(const std::string &fileName, const base::SpaceInformationPtr &si) {
	std::ifstream inputFile(fileName);
	if (!inputFile) {
		std::cerr << "Error: Could not open path file: " << fileName << std::endl;
		return;
	}

	std::cout << "\nChecking Path Validity from " << fileName << ":\n";
	std::string line;
	size_t stateIdx = 1;

	while (std::getline(inputFile, line)) {
		std::istringstream iss(line);
		double x, y, z, yaw;
		if (!(iss >> x >> y >> z >> yaw)) {
			std::cerr << "Invalid line format: " << line << std::endl;
			continue;
		}

		base::ScopedState<base::R3SO2StateSpace> state(si->getStateSpace());
		state->setX(x);
		state->setY(y);
		state->setZ(z);
		state->setYaw(yaw);

		bool isValid = si->isValid(state.get());
		std::cout << "State " << stateIdx++ << ": " << x << " " << y << " " << z << " " << yaw << " - " << (isValid ? "Valid" : "Invalid") << "\n"; 
	}
}

int main(int argc, char **argv) {
	if (argc != 3) {
		std::cerr << "Usage: " << argv[0] << " <config.file.txt> <path.file.txt>" << std::endl;
		return -1;
	}

	// Open the configuration file
    std::ifstream configFile(argv[1]);
    if (!configFile) {
        std::cerr << "Error: Could not open the configuration file." << std::endl;
        return -2;
    }

    // Create a map to store the configuration settings
    std::map<std::string, std::string> configSettings;

    // Read and parse the config file
    std::string line;
    while (std::getline(configFile, line)) {
        std::istringstream iss(line);
        std::string key, value;
        if (std::getline(iss, key, '=') && std::getline(iss, value)) {
            // Trim any whitespaces
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);
            configSettings[key] = value;
        }
    }

    // Plan in R3SO2
    app::R3SO2RigidBodyPlanning setup;

    // Load the robot and the environment
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

    // Define upper and lower bounds
    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, std::stod(configSettings["volume.max.x"]));
    bounds.setHigh(1, std::stod(configSettings["volume.max.y"]));
    bounds.setHigh(2, std::stod(configSettings["volume.max.z"]));
    bounds.setLow(0, std::stod(configSettings["volume.min.x"]));
    bounds.setLow(1, std::stod(configSettings["volume.min.y"]));
    bounds.setLow(2, std::stod(configSettings["volume.min.z"]));
    setup.getStateSpace()->as<base::R3SO2StateSpace>()->setBounds(bounds);

	// set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // Setting collision checking resolution
    setup.getSpaceInformation()->setStateValidityCheckingResolution(1e-6);


    // Call setup to initialize the state space and validity checker
    setup.setup();


    // Check the validity of the path from the given file
    checkPathValidityFromFile(argv[2], setup.getSpaceInformation());

    return 0;
}
