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
    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;

    geometry_msgs::Pose pose;
    tf::Quaternion q;
    std::vector<double> path_matrix[4];
    double start_roll, start_pitch, start_yaw;
    double end_roll, end_pitch, end_yaw;
    double Se = sqrt(pow((move_lin.end.position.x - move_lin.start.position.x), 2) +
                     pow((move_lin.end.position.y - move_lin.start.position.y), 2) +
                     pow((move_lin.end.position.z - move_lin.start.position.z), 2));

    // Convert Quaternions into RPY Angles for start and end pose
    q = tf::Quaternion(move_lin.start.orientation.x,
                       move_lin.start.orientation.y,
                       move_lin.start.orientation.z,
                       move_lin.start.orientation.w);

    tf::Matrix3x3(q).getRPY(start_roll, start_pitch, start_yaw);

    q = tf::Quaternion(move_lin.end.orientation.x,
                       move_lin.end.orientation.y,
                       move_lin.end.orientation.z,
                       move_lin.end.orientation.w);

    tf::Matrix3x3(q).getRPY(end_roll, end_pitch, end_yaw);

    // Calculate path length for the angular movement
    double Se_roll, Se_pitch, Se_yaw;
    Se_roll  = end_roll  - start_roll;
    Se_pitch = end_pitch - start_pitch;
    Se_yaw   = end_yaw   - start_yaw;

    // Calculate path for each Angle
    if(!TPG_.calculateProfileForAngularMovements(path_matrix, Se, Se_roll, Se_pitch, Se_yaw, move_lin))
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
        if(!move_lin.rotate_only)
        {
            pose.position.x = move_lin.start.position.x + linear_path.at(i) * (move_lin.end.position.x - move_lin.start.position.x) / Se;
            pose.position.y = move_lin.start.position.y + linear_path.at(i) * (move_lin.end.position.y - move_lin.start.position.y) / Se;
            pose.position.z = move_lin.start.position.z + linear_path.at(i) * (move_lin.end.position.z - move_lin.start.position.z) / Se;
        }
        else
        {
            pose.position.x = move_lin.start.position.x;
            pose.position.y = move_lin.start.position.y;
            pose.position.z = move_lin.start.position.z;
        }

        // Transform RPY to Quaternion
        q.setRPY(roll_path.at(i), pitch_path.at(i), yaw_path.at(i));

        // Get Quaternion Values
        pose.orientation.x = q.getX();
        pose.orientation.y = q.getY();
        pose.orientation.z = q.getZ();
        pose.orientation.w = q.getW();
        pose_vector.push_back(pose);
    }
    return true;
}

bool TrajectoryInterpolator::circularInterpolation(std::vector<geometry_msgs::Pose>& pose_vector,
                                                   cob_cartesian_controller::MoveCircStruct& move_circ)
{
    tf::Transform C, P, T;
    tf::Quaternion q;
    geometry_msgs::Pose pose, pos;
    std::vector<double> path_array;

    // Convert RPY angles into [RAD]
    move_circ.start_angle    *= M_PI/180;
    move_circ.end_angle      *= M_PI/180;
    move_circ.roll_center    *= M_PI/180;
    move_circ.pitch_center   *= M_PI/180;
    move_circ.yaw_center     *= M_PI/180;

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
    if(!TPG_.calculateProfile(path_array, Se, move_circ.profile.vel, move_circ.profile.accl, move_circ.profile.profile_type))
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

        // Fill the Pose
        pose.position.x = P.getOrigin().x();
        pose.position.y = P.getOrigin().y();
        pose.position.z = P.getOrigin().z();

        pose.orientation.x = P.getRotation()[0];
        pose.orientation.y = P.getRotation()[1];
        pose.orientation.z = P.getRotation()[2];
        pose.orientation.w = P.getRotation()[3];

        // Put the pose into the pose Vector
        pose_vector.push_back(pose);
    }
    return true;
}
