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

#include <cob_cartesian_controller/trajectory_interpolator/trajectory_interpolator.h>

bool TrajectoryInterpolator::linearInterpolation(std::vector<geometry_msgs::Pose>& pose_vector,
                                                 cob_cartesian_controller::MoveLinStruct& move_lin)
{
    tf::Quaternion q;
    double start_roll, start_pitch, start_yaw;
    double end_roll, end_pitch, end_yaw;

    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;
    std::vector<double> path_matrix[4];
    geometry_msgs::Pose pose;

    double Se = sqrt(pow((move_lin.end.position.x - move_lin.start.position.x), 2) +
                     pow((move_lin.end.position.y - move_lin.start.position.y), 2) +
                     pow((move_lin.end.position.z - move_lin.start.position.z), 2));

    // Convert Quaternions into RPY Angles for start and end pose
    tf::quaternionMsgToTF(move_lin.start.orientation, q);
    tf::Matrix3x3(q).getRPY(start_roll, start_pitch, start_yaw);

    tf::quaternionMsgToTF(move_lin.end.orientation, q);
    tf::Matrix3x3(q).getRPY(end_roll, end_pitch, end_yaw);

    // Calculate path length for the angular movement
    double Se_roll, Se_pitch, Se_yaw;
    Se_roll  = end_roll  - start_roll;
    Se_pitch = end_pitch - start_pitch;
    Se_yaw   = end_yaw   - start_yaw;

    // Calculate path for each Angle
    if(!trajectory_profile_generator_lin_.calculateProfile(path_matrix, Se, Se_roll, Se_pitch, Se_yaw, move_lin))
    {
        return false;
    }

    linear_path  = path_matrix[0];
    roll_path    = path_matrix[1];
    pitch_path   = path_matrix[2];
    yaw_path     = path_matrix[3];

    // Interpolate the linear path
    for(int i = 0 ; i < path_matrix[0].size() ; i++)
    {
        if(move_lin.rotate_only)
        {
            pose.position.x = move_lin.start.position.x;
            pose.position.y = move_lin.start.position.y;
            pose.position.z = move_lin.start.position.z;
        }
        else
        {
            pose.position.x = move_lin.start.position.x + linear_path.at(i) * (move_lin.end.position.x - move_lin.start.position.x) / Se;
            pose.position.y = move_lin.start.position.y + linear_path.at(i) * (move_lin.end.position.y - move_lin.start.position.y) / Se;
            pose.position.z = move_lin.start.position.z + linear_path.at(i) * (move_lin.end.position.z - move_lin.start.position.z) / Se;
        }

        // Transform RPY to Quaternion
        q.setRPY(roll_path.at(i), pitch_path.at(i), yaw_path.at(i));
        tf::quaternionTFToMsg(q, pose.orientation);

        pose_vector.push_back(pose);
    }
    return true;
}

bool TrajectoryInterpolator::circularInterpolation(std::vector<geometry_msgs::Pose>& pose_vector,
                                                   cob_cartesian_controller::MoveCircStruct& move_circ)
{
    tf::Quaternion q;
    tf::Transform C, P, T;

    std::vector<double> path_array;
    geometry_msgs::Pose pose;

    double Se = move_circ.end_angle - move_circ.start_angle;
    bool forward;

    if(Se < 0)
    {
        forward = false;
    }
    else
    {
        forward = true;
    }

    Se = std::fabs(Se);

    // Calculates the Path with RAMP or SINOID profile
    if(!trajectory_profile_generator_circ_.calculateProfile(path_array, Se, move_circ.profile))
    {
        return false;
    }

    // Define Center Pose
    C.setOrigin(tf::Vector3(move_circ.x_center, move_circ.y_center, move_circ.z_center));
    q.setRPY(move_circ.roll_center, move_circ.pitch_center, move_circ.yaw_center);
    C.setRotation(q);

    // Interpolate the circular path
    for(int i = 0 ; i < path_array.size() ; i++)
    {
        // Rotate T
        T.setOrigin(tf::Vector3(cos(path_array.at(i)) * move_circ.radius, 0, sin(path_array.at(i)) * move_circ.radius));

        if(forward)
        {
            T.setOrigin(tf::Vector3(cos(path_array.at(i)) * move_circ.radius, 0, sin(path_array.at(i)) * move_circ.radius));
            q.setRPY(0, -path_array.at(i), 0);
        }
        else
        {
            T.setOrigin(tf::Vector3(cos(move_circ.start_angle - path_array.at(i)) * move_circ.radius, 0, sin(move_circ.start_angle - path_array.at(i)) * move_circ.radius));
            q.setRPY(0, path_array.at(i), 0);
        }

        T.setRotation(q);

        // Calculate TCP Position
        P = C * T;
        
        tf::pointTFToMsg(P.getOrigin(), pose.position);
        tf::quaternionTFToMsg(P.getRotation(), pose.orientation);

        pose_vector.push_back(pose);
    }
    return true;
}
