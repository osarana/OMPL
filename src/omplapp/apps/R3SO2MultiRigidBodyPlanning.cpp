/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2012, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Joe Masterjohn */

#include "omplapp/apps/R3SO2MultiRigidBodyPlanning.h"

ompl::app::R3SO2MultiRigidBodyPlanning::R3SO2MultiRigidBodyPlanning(unsigned int n) :
    AppBase<AppType::GEOMETRIC>(std::make_shared<base::CompoundStateSpace>(), Motion_4D), n_(n)
{
    assert (n > 0);
    name_ = "Multi rigid body planning (R^3 x SO(2))";
    // Adding n R^3 x SO(2) StateSpaces
    for (unsigned int i = 0; i < n_; ++i)
        si_->getStateSpace()->as<base::CompoundStateSpace>()->addSubspace(
            std::make_shared<base::R3SO2StateSpace>(), 1.0);
}

void ompl::app::R3SO2MultiRigidBodyPlanning::inferEnvironmentBounds()
{
    // Infer bounds for all n R^3 x SO(2) spaces
    for (unsigned int i = 0; i < n_; ++i)
        InferEnvironmentBounds(getGeometricComponentStateSpace(i), *static_cast<RigidBodyGeometry*>(this));
}

void ompl::app::R3SO2MultiRigidBodyPlanning::inferProblemDefinitionBounds()
{
    // Make sure that all n R^3 x SO(2) spaces get the same bounds, if they are adjusted
    for (unsigned int i = 0; i < n_; ++i)
        InferProblemDefinitionBounds(AppTypeSelector<AppType::GEOMETRIC>::SimpleSetup::getProblemDefinition(),
                                    getGeometricStateExtractor(), factor_, add_,
                                    n_, getGeometricComponentStateSpace(i), mtype_);
}

ompl::base::ScopedState<> ompl::app::R3SO2MultiRigidBodyPlanning::getDefaultStartState() const
{
    base::ScopedState<> st(getStateSpace());
    auto* c_st = st->as<base::CompoundStateSpace::StateType>();
    for (unsigned int i = 0; i < n_; ++i)
    {
        aiVector3D s = getRobotCenter(i);
        auto* sub = c_st->as<base::R3SO2StateSpace::StateType>(i);
        sub->setX(s.x);
        sub->setY(s.y);
        sub->setZ(s.z);
        sub->setYaw(0.0);
    }
    return st;
}

const ompl::base::State* ompl::app::R3SO2MultiRigidBodyPlanning::getGeometricComponentStateInternal(const ompl::base::State* state, unsigned int index) const
{
    assert (index < n_);
    const auto* st = state->as<base::CompoundStateSpace::StateType>()->as<base::R3SO2StateSpace::StateType>(index);
    return static_cast<const base::State*>(st);
}
