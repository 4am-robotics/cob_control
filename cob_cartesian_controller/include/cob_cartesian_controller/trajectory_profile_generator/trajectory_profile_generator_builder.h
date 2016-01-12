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
 *   Builder class for generic profile generator.
 *
 ****************************************************************/

#ifndef COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_BUILDER_H
#define COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_BUILDER_H

#include <cob_cartesian_controller/cartesian_controller_data_types.h>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_base.h>

/* BEGIN TrajectoryProfileBuilder *****************************************************************************************/
class TrajectoryProfileBuilder
{
public:
    TrajectoryProfileBuilder() {}
    ~TrajectoryProfileBuilder() {}

    static TrajectoryProfileBase* createProfile(const cob_cartesian_controller::CartesianActionStruct& params);
};
/* END TrajectoryGeneratorBuilder *******************************************************************************************/

#endif  // COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_BUILDER_H
