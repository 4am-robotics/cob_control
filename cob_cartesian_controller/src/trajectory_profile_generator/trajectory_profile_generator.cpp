
/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This module contains the implementation of all interface types.
 *
 ****************************************************************/
#define RAMP_PROFILE 1
#define SINOID_PROFILE 2
#include "cob_cartesian_controller/trajectory_profile_generator/trajactory_profile_generator.h"
#include "cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_ramp.h"
#include "cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_sinoid.h"

/* BEGIN TrajectoryProfileBase *****************************************************************************************/

TrajectoryProfileBase* TrajectoryProfileBuilder::createProfile(const cob_cartesian_controller::CartesianActionStruct& params)
{
    TrajectoryProfileBase* ib = NULL;
    switch(params.profile.profile_type)
    {
        case RAMP_PROFILE:
            ib = new TrajectoryProfileRamp(params);
            break;
        case SINOID_PROFILE:
            ib = new TrajectoryProfileSinoid(params);
            break;
        default:
            ROS_ERROR("Unknown Profile");
            break;
    }

    return ib;
}
/* END TrajectoryProfileBase *******************************************************************************************/
