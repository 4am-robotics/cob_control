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
        double x, y, z, roll, pitch, yaw;
        geometry_msgs::Pose start, end;
        bool rotate_only;
        
        ProfileStruct profile;
    };

    struct MoveCircStruct
    {
        double x_center, y_center, z_center, roll_center, pitch_center, yaw_center;
        double start_angle, end_angle, radius;
        bool rotate_only;
        
        ProfileStruct profile;
    };

    struct CartesianActionStruct
    {
        std::string name;
        MoveLinStruct move_lin;
        MoveCircStruct move_circ;
    };

}//namespace

class errorException: public std::runtime_error
{
public:
    errorException(const std::string error_description) : std::runtime_error(error_description) { ; };
};

#endif /* COB_CARTESIAN_CONTROLLER_DATA_STRUCTURES_H_ */
