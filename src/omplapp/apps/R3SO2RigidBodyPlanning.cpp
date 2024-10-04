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

#include "omplapp/apps/R3SO2RigidBodyPlanning.h"

ompl::base::ScopedState<> ompl::app::R3SO2RigidBodyPlanning::getDefaultStartState() const
{
    base::ScopedState<base::R3SO2StateSpace> st(getGeometricComponentStateSpace());
    aiVector3D s = getRobotCenter(0);

    st->setX(s.x);
    st->setY(s.y);
    st->setZ(s.z);
    st->setYaw(0.0);

    return st;
}
