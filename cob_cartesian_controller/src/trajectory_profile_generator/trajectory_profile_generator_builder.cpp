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
 *   ROS package name: cob_cartesian_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: December, 2015
 *
 * \brief
 *   This module contains the implementation of all profile types.
 *
 ****************************************************************/

#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_builder.h>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_ramp.h>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_sinoid.h>
#include <cob_cartesian_controller/Profile.h>

/* BEGIN TrajectoryProfileBuilder *****************************************************************************************/
TrajectoryProfileBase* TrajectoryProfileBuilder::createProfile(const cob_cartesian_controller::CartesianActionStruct& params)
{
    cob_cartesian_controller::Profile msg;

    const int RAMP = static_cast<const int>(msg.RAMP);
    const int SINOID = static_cast<const int>(msg.SINOID);

    TrajectoryProfileBase* ib = NULL;
    switch (params.profile.profile_type)
    {
        case RAMP:
            ib = new TrajectoryProfileRamp(params);
            break;
        case SINOID:
            ib = new TrajectoryProfileSinoid(params);
            break;
        default:
            ROS_ERROR("Unknown Profile");
            break;
    }

    return ib;
}
/* END TrajectoryProfileBuilder *******************************************************************************************/
