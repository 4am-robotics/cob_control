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

#ifndef COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_HELPER_CLASSES_DATA_STRUCTURES_H_
#define COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_HELPER_CLASSES_DATA_STRUCTURES_H_

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <exception>

struct trajectory_action_move_lin{
        bool rotate_only;
        std::string profile;
        double x, y, z, roll, pitch, yaw, vel, accl;
        geometry_msgs::Pose start, end;
};

struct trajectory_action_move_circ{
        bool rotateOnly;
        std::string profile;
        double x_center, y_center, z_center, roll_center, pitch_center, yaw_center, vel, accl, startAngle, endAngle, radius;
};

struct trajectory_action{
        std::string name;
        double hold_time;
        trajectory_action_move_lin move_lin;
        trajectory_action_move_circ move_circ;
};

class errorException: public std::runtime_error
{
public:
        errorException(const std::string errorDescription) : std::runtime_error(errorDescription) { ; };
};

#endif /* COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_HELPER_CLASSES_DATA_STRUCTURES_H_ */
