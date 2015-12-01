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
    this->trajectory_profile_generator_.reset(TrajectoryProfileBuilder::createProfile(as));

    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = root_frame_;

    tf::Quaternion q_start, q_end;

    std::vector<double> linear_path, angular_path, path;
    std::vector<double> path_matrix[2];
    geometry_msgs::Pose pose;

    double norm_factor;
    tf::quaternionMsgToTF(as.move_lin.start.orientation,q_start);
    tf::quaternionMsgToTF(as.move_lin.end.orientation,q_end);

    double Se_lin = sqrt(pow((as.move_lin.end.position.x - as.move_lin.start.position.x), 2) +
                     pow((as.move_lin.end.position.y - as.move_lin.start.position.y), 2) +
                     pow((as.move_lin.end.position.z - as.move_lin.start.position.z), 2));

    double Se_rot = q_start.angleShortestPath(q_end);

    ROS_INFO_STREAM("Se_lin: " << Se_lin << "  Se_rot: " << Se_rot);
    // Calculate path for each Angle
//    (std::vector<double> path_matrix[2],const double Se_lin, const double Se_rot,geometry_msgs::Pose start)

    if(!trajectory_profile_generator_->calculateProfile(path_matrix, Se_lin, Se_rot, as.move_lin.start))
    {
        return false;
    }

    linear_path  = path_matrix[0];
    angular_path = path_matrix[1];

    if(linear_path.back() == 0)
    {
        norm_factor = 1/angular_path.back();
        path = angular_path;
    }
    else
    {
        norm_factor = 1/linear_path.back();
        path = linear_path;
    }

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

        tf::quaternionTFToMsg(q_start.slerp(q_end, path.at(i) * norm_factor), pose.orientation);
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

    tf::Quaternion q_start, q_end;
    tf::Quaternion q;
    tf::Transform C, P, T;

    double norm_factor;
    tf::quaternionMsgToTF(as.move_lin.start.orientation,q_start);
    tf::quaternionMsgToTF(as.move_lin.end.orientation,q_end);

    std::vector<double> linear_path, angular_path, path;

    std::vector<double> path_array;
    std::vector<double> path_matrix[2];

    geometry_msgs::Pose pose;

    double Se_rot = as.move_circ.end_angle - as.move_circ.start_angle;

    Se_rot = std::fabs(Se_rot);

    // Calculates the Path with RAMP or SINOID profile
    if(!this->trajectory_profile_generator_->calculateProfile(path_matrix, Se_rot, 0, pose))
    {
        return false;
    }

    linear_path  = path_matrix[0];
    angular_path = path_matrix[1];

    // Define Center Pose
    C.setOrigin(tf::Vector3(as.move_circ.pose_center.position.x, as.move_circ.pose_center.position.y, as.move_circ.pose_center.position.z));
    tf::quaternionMsgToTF(as.move_circ.pose_center.orientation, q);
    C.setRotation(q);

    // Interpolate the circular path
    for(int i = 0 ; i < linear_path.size() ; i++)
    {
        ROS_INFO_STREAM("linear_path.at(" << i << "): " << linear_path.at(i));

        // Rotate T
        T.setOrigin(tf::Vector3(cos(linear_path.at(i)) * as.move_circ.radius, 0, sin(linear_path.at(i)) * as.move_circ.radius));
        q.setRPY(0, path_array.at(i), 0);
        T.setRotation(q);

        // Calculate TCP Position
        P = C * T;

        tf::pointTFToMsg(P.getOrigin(), pose.position);
        tf::quaternionTFToMsg(P.getRotation(), pose.orientation);

        pose_array.poses.push_back(pose);
    }
    return true;
}
