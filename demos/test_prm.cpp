#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

int main() {
    // Define a 2D state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);

    // Set bounds for the state space
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);
    space->setBounds(bounds);

    // Create a SimpleSetup object
    og::SimpleSetup ss(space);

    // Set the state validity checker
    ss.setStateValidityChecker([](const ob::State *state) {
        const auto *realState = state->as<ob::RealVectorStateSpace::StateType>();
        double x = realState->values[0];
        double y = realState->values[1];
        return (x * x + y * y >= 0.25); // Example: States outside a circle of radius 0.5 are valid
    });

    // Specify start and goal states
    ob::ScopedState<> start(space);
    start[0] = 0.1;
    start[1] = 0.1;

    ob::ScopedState<> goal(space);
    goal[0] = 0.9;
    goal[1] = 0.9;

    ss.setStartAndGoalStates(start, goal);

    // Set the PRM planner
    ss.setPlanner(std::make_shared<og::PRM>(ss.getSpaceInformation()));

    // Solve the problem
    if (ss.solve(1.0)) {
        std::cout << "Found solution:" << std::endl;
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    } else {
        std::cout << "No solution found." << std::endl;
    }

    return 0;
}
