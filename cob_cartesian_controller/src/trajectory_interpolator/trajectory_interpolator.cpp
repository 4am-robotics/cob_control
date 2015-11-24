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
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_builder.h>
#include <math.h>

bool TrajectoryInterpolator::linearInterpolation(geometry_msgs::PoseArray& pose_array,
                                                 const cob_cartesian_controller::CartesianActionStruct& as)
{
    double Se_roll, Se_pitch, Se_yaw;
    this->trajectory_profile_generator_.reset(TrajectoryProfileBuilder::createProfile(as));

    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = root_frame_;

    tf::Quaternion q_start, q_end, q_relative, q;
    double start_roll, start_pitch, start_yaw;
    double end_roll, end_pitch, end_yaw;

    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;
    std::vector<double> path_matrix[4];
    geometry_msgs::Pose pose;

    double Se = sqrt(pow((as.move_lin.end.position.x - as.move_lin.start.position.x), 2) +
                     pow((as.move_lin.end.position.y - as.move_lin.start.position.y), 2) +
                     pow((as.move_lin.end.position.z - as.move_lin.start.position.z), 2));

//    // Convert Quaternions into RPY Angles for start and end pose
    tf::quaternionMsgToTF(as.move_lin.start.orientation, q_start);
    tf::Matrix3x3(q_start).getRPY(start_roll, start_pitch, start_yaw);

    ROS_INFO_STREAM("q_start_w: " << q_start.getW() << " q_start_x: " << q_start.getX() << " q_start_y: " << q_start.getY() << " q_start_z: " << q_start.getZ());
//    ROS_INFO_STREAM("start_roll: " << start_roll << " start_pitch: " << start_pitch << " start_yaw: " << start_yaw);

    tf::quaternionMsgToTF(as.move_lin.end.orientation, q_end);
    tf::Matrix3x3(q_end).getRPY(end_roll, end_pitch, end_yaw);

    ROS_INFO_STREAM("q_end_w: " << q_end.getW() << " q_end_x: " << q_end.getX() << " q_end_y: " << q_end.getY() << " q_end_z: " << q_end.getZ());
//    ROS_INFO_STREAM("end_roll: " << end_roll << " end_pitch: " << end_pitch << " end_yaw: " << end_yaw);


//    q_relative = q_start.slerp(q_end, 1);
//    ROS_INFO_STREAM("q_slerp_w: " << q_relative.getW() << " q_slerp_x: " << q_relative.getX() << " q_slerp_y: " << q_relative.getY() << " q_slerp_z: " << q_relative.getZ());

    q_start = q_start.normalized();
    q_end = q_end.normalize();
    q_relative = q_end * q_start.inverse();

    tf::Matrix3x3(q_relative).getRPY(Se_roll, Se_pitch, Se_yaw);

//    if(q_start.dot(q_end) > 1-0.0001)
//    {
//        Se_pitch = Se_roll = Se_yaw = 0;
//    }
//    q_relative = q_end * q_start.inverse();
//    tf::Matrix3x3(q_relative).getRPY(Se_roll, Se_pitch, Se_yaw);

//    ROS_INFO_STREAM("q_relative_w: " << q_relative.getW() << " q_relative_x: " << q_relative.getX() << " q_relative_y: " << q_relative.getY() << " q_relative_z: " << q_relative.getZ());
//    ROS_INFO_STREAM("Se_roll: " << Se_roll << " Se_pitch: " << Se_pitch << " Se_yaw: " << Se_yaw);

    // Calculate path length for the angular movement
    //    Se_roll  = std::fabs(end_roll)  - std::fabs(start_roll);
    //    Se_pitch = std::fabs(end_pitch) - std::fabs(start_pitch);
    //    Se_yaw   = std::fabs(end_yaw)   - std::fabs(start_yaw);

//    Se_roll  = end_roll  - start_roll;
//    Se_pitch = end_pitch - start_pitch;
//    Se_yaw   = end_yaw   - start_yaw;

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
    for(unsigned int i = 0 ; i < path_matrix[0].size() ; i++)
    {
        if(linear_path.back() == 0)
        {
            pose.position.x = as.move_lin.start.position.x;
            pose.position.y = as.move_lin.start.position.y;
            pose.position.z = as.move_lin.start.position.z;
        }
        else
        {
            pose.position.x = as.move_lin.start.position.x + linear_path.at(i) * (as.move_lin.end.position.x - as.move_lin.start.position.x) / linear_path.back();
            pose.position.y = as.move_lin.start.position.y + linear_path.at(i) * (as.move_lin.end.position.y - as.move_lin.start.position.y) / linear_path.back();
            pose.position.z = as.move_lin.start.position.z + linear_path.at(i) * (as.move_lin.end.position.z - as.move_lin.start.position.z) / linear_path.back();

        }

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
