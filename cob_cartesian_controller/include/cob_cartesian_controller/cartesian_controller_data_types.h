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
 * \date Date of creation: July, 2015
 *
 * \brief
 *   ...
 *
 ****************************************************************/

#ifndef COB_CARTESIAN_CONTROLLER_DATA_STRUCTURES_H_
#define COB_CARTESIAN_CONTROLLER_DATA_STRUCTURES_H_

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <exception>

namespace cob_cartesian_controller
{
    struct ProfileStruct
    {
        unsigned int profile_type;
        double vel, accl;
    };

    struct MoveLinStruct
    {
        geometry_msgs::Pose start, end;
        bool rotate_only;

        ProfileStruct profile;
    };

    struct MoveCircStruct
    {
        geometry_msgs::Pose pose_center;
        double start_angle, end_angle;
        double radius;
        bool rotate_only;

        ProfileStruct profile;
    };

    struct CartesianActionStruct
    {
        unsigned int move_type;
        MoveLinStruct move_lin;
        MoveCircStruct move_circ;
    };

}//namespace


#endif /* COB_CARTESIAN_CONTROLLER_DATA_STRUCTURES_H_ */
