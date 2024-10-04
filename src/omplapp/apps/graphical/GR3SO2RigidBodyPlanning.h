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

#ifndef OMPLAPP_G_R3SO2_RIGID_BODY_PLANNING_
#define OMPLAPP_G_R3SO2_RIGID_BODY_PLANNING_

#include "omplapp/apps/R3SO2RigidBodyPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GR3SO2RigidBodyPlanning : public R3SO2RigidBodyPlanning,
                                      public RenderGeometry
        {
        public:

            GR3SO2RigidBodyPlanning(void) : R3SO2RigidBodyPlanning(),
                                          RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }

            virtual ~GR3SO2RigidBodyPlanning(void)
            {
            }
        };

    }
}

#endif
