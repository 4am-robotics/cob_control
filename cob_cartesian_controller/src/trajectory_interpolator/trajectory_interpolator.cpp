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
#include <cob_cartesian_controller/trajectory_profile_generator/trajactory_profile_generator.h>

bool TrajectoryInterpolator::linearInterpolation(geometry_msgs::PoseArray& pose_array,
                                                 const cob_cartesian_controller::CartesianActionStruct& as)
{

    this->trajectory_profile_generator_.reset(TrajectoryProfileBuilder::createProfile(as));

    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = root_frame_;

    tf::Quaternion q;
    double start_roll, start_pitch, start_yaw;
    double end_roll, end_pitch, end_yaw;

    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;
    std::vector<double> path_matrix[4];
    geometry_msgs::Pose pose;

    double Se = sqrt(pow((as.move_lin.end.position.x - as.move_lin.start.position.x), 2) +
                     pow((as.move_lin.end.position.y - as.move_lin.start.position.y), 2) +
                     pow((as.move_lin.end.position.z - as.move_lin.start.position.z), 2));

    // Convert Quaternions into RPY Angles for start and end pose
    tf::quaternionMsgToTF(as.move_lin.start.orientation, q);
    tf::Matrix3x3(q).getRPY(start_roll, start_pitch, start_yaw);

    tf::quaternionMsgToTF(as.move_lin.end.orientation, q);
    tf::Matrix3x3(q).getRPY(end_roll, end_pitch, end_yaw);

    // Calculate path length for the angular movement
    double Se_roll, Se_pitch, Se_yaw;
    Se_roll  = end_roll  - start_roll;
    Se_pitch = end_pitch - start_pitch;
    Se_yaw   = end_yaw   - start_yaw;

    ROS_INFO_STREAM("Se: " << Se << "   Se_roll: " << Se_roll << "   Se_pitch: " << Se_pitch << "   Se_yaw: " << Se_yaw);

    // Calculate path for each Angle
    if(!this->trajectory_profile_generator_->calculateProfile(path_matrix, Se, Se_roll, Se_pitch, Se_yaw, as.move_lin.start))
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
        pose.position.x = as.move_lin.start.position.x + linear_path.at(i) * (as.move_lin.end.position.x - as.move_lin.start.position.x) / Se;
        pose.position.y = as.move_lin.start.position.y + linear_path.at(i) * (as.move_lin.end.position.y - as.move_lin.start.position.y) / Se;
        pose.position.z = as.move_lin.start.position.z + linear_path.at(i) * (as.move_lin.end.position.z - as.move_lin.start.position.z) / Se;

        // Transform RPY to Quaternion
        q.setRPY(roll_path.at(i), pitch_path.at(i), yaw_path.at(i));
        tf::quaternionTFToMsg(q, pose.orientation);

        pose_array.poses.push_back(pose);
    }
    return true;
}

bool TrajectoryInterpolator::circularInterpolation(geometry_msgs::PoseArray& pose_array,
                                                   const cob_cartesian_controller::CartesianActionStruct& as)
{
    this->trajectory_profile_generator_.reset(TrajectoryProfileBuilder::createProfile(as));

    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = root_frame_;

    tf::Quaternion q;
    tf::Transform C, P, T;

    std::vector<double> path_array;
    std::vector<double> path_matrix[4];

    geometry_msgs::Pose pose;

    double Se = as.move_circ.end_angle - as.move_circ.start_angle;

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
    if(!this->trajectory_profile_generator_->calculateProfile(path_matrix, Se, 0, 0, 0, pose))
    {
        return false;
    }

    path_array = path_matrix[0];
    // Define Center Pose
    C.setOrigin(tf::Vector3(as.move_circ.pose_center.position.x, as.move_circ.pose_center.position.y, as.move_circ.pose_center.position.z));
    tf::quaternionMsgToTF(as.move_circ.pose_center.orientation, q);
    C.setRotation(q);

    // Interpolate the circular path
    for(int i = 0 ; i < path_array.size() ; i++)
    {
        // Rotate T
        T.setOrigin(tf::Vector3(cos(path_array.at(i)) * as.move_circ.radius, 0, sin(path_array.at(i)) * as.move_circ.radius));

        if(forward)
        {
            T.setOrigin(tf::Vector3(cos(path_array.at(i)) * as.move_circ.radius, 0, sin(path_array.at(i)) * as.move_circ.radius));
            q.setRPY(0, -path_array.at(i), 0);
        }
        else
        {
            T.setOrigin(tf::Vector3(cos(as.move_circ.start_angle - path_array.at(i)) * as.move_circ.radius, 0, sin(as.move_circ.start_angle - path_array.at(i)) * as.move_circ.radius));
            q.setRPY(0, path_array.at(i), 0);
        }

        T.setRotation(q);

        // Calculate TCP Position
        P = C * T;

        tf::pointTFToMsg(P.getOrigin(), pose.position);
        tf::quaternionTFToMsg(P.getRotation(), pose.orientation);

        pose_array.poses.push_back(pose);
    }
    return true;
}
