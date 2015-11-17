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
 * \date Date of creation: September, 2015
 *
 * \brief
 *
 ****************************************************************/
#ifndef TRAJECTORY_PROFILE_BUILDER_H_
#define TRAJECTORY_PROFILE_BUILDER_H_

#include "cob_cartesian_controller/cartesian_controller_data_types.h"
#include "cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_base.h"


/* BEGIN TrajectoryProfileBuilder *****************************************************************************************/
class TrajectoryProfileBuilder
{
    public:
        TrajectoryProfileBuilder() {}
        ~TrajectoryProfileBuilder() {}

        static TrajectoryProfileBase* createProfile(const cob_cartesian_controller::CartesianActionStruct& params);
};
/* END TrajectoryGeneratorBuilder *******************************************************************************************/

#endif
