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

#ifndef OMPLAPP_R3SO2_RIGID_BODY_PLANNING_
#define OMPLAPP_R3SO2_RIGID_BODY_PLANNING_

#include <ompl/base/spaces/R3SO2StateSpace.h>

#include "omplapp/apps/AppBase.h"

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in R^3 x SO(2). */
        class R3SO2RigidBodyPlanning : public AppBase<AppType::GEOMETRIC>
        {
        public:
            R3SO2RigidBodyPlanning() : AppBase<AppType::GEOMETRIC>(std::make_shared<base::R3SO2StateSpace>(), Motion_4D)
            {
                name_ = "Rigid body planning (R^3 x SO(2))";
            }

            ~R3SO2RigidBodyPlanning() override = default;

            bool isSelfCollisionEnabled() const override
            {
                return false;
            }

            base::ScopedState<> getDefaultStartState() const override;

            base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const override
            {
                return state;
            }

            const base::StateSpacePtr& getGeometricComponentStateSpace() const override
            {
                return getStateSpace();
            }

            unsigned int getRobotCount() const override
            {
                return 1;
            }

        protected:

            const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const override
            {
                return state;
            }

        };

    }
}

#endif
