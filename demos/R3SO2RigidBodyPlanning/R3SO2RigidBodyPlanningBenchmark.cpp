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

#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/R3SO2RigidBodyPlanning.h>
#include <omplapp/config.h>

#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
using namespace ompl;

void benchmark0(std::string &benchmark_name, app::R3SO2RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    benchmark_name = std::string("cubicles");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::R3SO2StateSpace> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(-40.62);
    start->setZ(70.57);
    start->setYaw(0.0);

    base::ScopedState<base::R3SO2StateSpace> goal(start);
    goal->setX(200.49);
    goal->setY(-40.62);
    goal->setZ(70.57);
    goal->setYaw(0.0);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);
    setup.setup();

    std::vector<double> cs(3);
    cs[0] = 35;
    cs[1] = 35;
    cs[2] = 35;
    setup.getStateSpace()->getDefaultProjection()->setCellSizes(cs);

    runtime_limit = 10.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 500;
}

void benchmark1(std::string &benchmark_name, app::R3SO2RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    benchmark_name = std::string("Apartment");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::R3SO2StateSpace> start(setup.getSpaceInformation());
    start->setX(-31.19);
    start->setY(-99.85);
    start->setZ(36.46);
    start->setYaw(0.0);

    base::ScopedState<base::R3SO2StateSpace> goal(start);
    goal->setX(116.81);
    goal->setY(-99.85);
    goal->setZ(36.46);
    goal->setYaw(0.0);

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 295.77);
    bounds.setHigh(1, 168.26);
    bounds.setHigh(2, 90.39);
    bounds.setLow(0, -73.76);
    bounds.setLow(1, -179.59);
    bounds.setLow(2, -0.03);
    setup.getStateSpace()->as<base::R3SO2StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 60.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 50;
}

void preRunEvent(const base::PlannerPtr & /*planner*/)
{
}

void postRunEvent(const base::PlannerPtr & /*planner*/, tools::Benchmark::RunProperties & /*run*/)
{
}

int main(int argc, char **argv)
{
    app::R3SO2RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;
    int benchmark_id = argc > 1 ? ((argv[1][0] - '0') % 2) : 0;

    if (benchmark_id == 0)
        benchmark0(benchmark_name, setup, runtime_limit, memory_limit, run_count);
    else
        benchmark1(benchmark_name, setup, runtime_limit, memory_limit, run_count);

    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    // optionally set pre & pos run events
    b.setPreRunEvent([](const base::PlannerPtr &planner) { preRunEvent(planner); });
    b.setPostRunEvent(
        [](const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run) { postRunEvent(planner, run); });

    b.addPlanner(std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::BKPIECE1>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::LBKPIECE1>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::KPIECE1>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::SBL>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));

    int sampler_id = argc > 2 ? ((argv[2][0] - '0') % 5) : -1;

    if (sampler_id == 0 || sampler_id < 0)
    {
        // run all planners with a uniform valid state sampler on the benchmark problem
        setup.getSpaceInformation()->setValidStateSamplerAllocator(
            [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
                return std::make_shared<base::UniformValidStateSampler>(si);
            });
        b.addExperimentParameter("sampler_id", "INTEGER", "0");
        b.benchmark(request);
        b.saveResultsToFile();
    }

    if (sampler_id == 1 || sampler_id < 0)
    {
        // run all planners with a Gaussian valid state sampler on the benchmark problem
        setup.getSpaceInformation()->setValidStateSamplerAllocator(
            [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
                return std::make_shared<base::GaussianValidStateSampler>(si);
            });
        b.addExperimentParameter("sampler_id", "INTEGER", "1");
        b.benchmark(request);
        b.saveResultsToFile();
    }

    if (sampler_id == 2 || sampler_id < 0)
    {
        // run all planners with a obstacle-based valid state sampler on the benchmark problem
        setup.getSpaceInformation()->setValidStateSamplerAllocator(
            [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
                return std::make_shared<base::ObstacleBasedValidStateSampler>(si);
            });
        b.addExperimentParameter("sampler_id", "INTEGER", "2");
        b.benchmark(request);
        b.saveResultsToFile();
    }

    if (sampler_id == 3 || sampler_id < 0)
    {
        // run all planners with a maximum-clearance valid state sampler on the benchmark problem
        setup.getSpaceInformation()->setValidStateSamplerAllocator(
            [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
                auto vss = std::make_shared<base::MaximizeClearanceValidStateSampler>(si);
                vss->setNrImproveAttempts(5);
                return vss;
            });
        b.addExperimentParameter("sampler_id", "INTEGER", "3");
        b.benchmark(request);
        b.saveResultsToFile();
    }

    if (sampler_id == 4 || sampler_id < 0)
    {
        // run all planners with a bridge-test valid state sampler on the benchmark problem
        setup.getSpaceInformation()->setValidStateSamplerAllocator(
            [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
                return std::make_shared<base::BridgeTestValidStateSampler>(si);
            });
        b.addExperimentParameter("sampler_id", "INTEGER", "4");
        b.benchmark(request);
        b.saveResultsToFile();
    }
    return 0;
}
