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
 *   Helper functions  used in the cob_cartesian_controller package.
 *
 ****************************************************************/

#include <string>
#include <vector>
#include <algorithm>
#include <cob_cartesian_controller/cartesian_controller_utils.h>

geometry_msgs::Pose CartesianControllerUtils::getPose(const std::string& target_frame, const std::string& source_frame)
{
    geometry_msgs::Pose pose;
    tf::StampedTransform stamped_transform;

    stamped_transform = getStampedTransform(target_frame, source_frame);

    tf::pointTFToMsg(stamped_transform.getOrigin(), pose.position);
    tf::quaternionTFToMsg(stamped_transform.getRotation(), pose.orientation);

    return pose;
}

tf::StampedTransform CartesianControllerUtils::getStampedTransform(const std::string& target_frame, const std::string& source_frame)
{
    tf::StampedTransform stamped_transform;
    bool transform = false;

    do
    {
        try
        {
            ros::Time now = ros::Time::now();
            tf_listener_.waitForTransform(target_frame, source_frame, now, ros::Duration(0.1));
            tf_listener_.lookupTransform(target_frame, source_frame, now, stamped_transform);
            transform = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("CartesianControllerUtils::getStampedTransform: \n%s", ex.what());
            ros::Duration(0.1).sleep();
        }
    } while (!transform && ros::ok());

    return stamped_transform;
}

void CartesianControllerUtils::transformPose(const std::string source_frame, const std::string target_frame, const geometry_msgs::Pose pose_in, geometry_msgs::Pose& pose_out)
{
    bool transform = false;
    geometry_msgs::PoseStamped stamped_in, stamped_out;
    stamped_in.header.frame_id = source_frame;
    stamped_in.pose = pose_in;

    do
    {
        try
        {
            ros::Time now = ros::Time::now();
            tf_listener_.waitForTransform(target_frame, source_frame, now, ros::Duration(0.1));
            tf_listener_.transformPose(target_frame, stamped_in, stamped_out);
            pose_out = stamped_out.pose;
            transform = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("CartesianControllerUtils::transformPose: \n%s", ex.what());
            ros::Duration(0.1).sleep();
        }
    } while (!transform && ros::ok());
}


/// Used to check whether the chain_tip_link is close to the target_frame
/// 'stamped_transform' expreses the transform between the two frames.
/// Thus inEpsilonArea() returns 'true' in case 'stamped_transform' is "smaller" than 'epsilon'
bool CartesianControllerUtils::inEpsilonArea(const tf::StampedTransform& stamped_transform, const double epsilon)
{
    double roll, pitch, yaw;
    stamped_transform.getBasis().getRPY(roll, pitch, yaw);

    bool x_okay, y_okay, z_okay = false;
    bool roll_okay, pitch_okay, yaw_okay = false;

    x_okay      = std::fabs(stamped_transform.getOrigin().x()) < epsilon;
    y_okay      = std::fabs(stamped_transform.getOrigin().y()) < epsilon;
    z_okay      = std::fabs(stamped_transform.getOrigin().z()) < epsilon;

    roll_okay   = std::fabs(roll)  < epsilon;
    pitch_okay  = std::fabs(pitch) < epsilon;
    yaw_okay    = std::fabs(yaw)   < epsilon;

    if (x_okay && y_okay && z_okay && roll_okay && pitch_okay && yaw_okay)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void CartesianControllerUtils::poseToRPY(const geometry_msgs::Pose& pose, double& roll, double& pitch, double& yaw)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

void CartesianControllerUtils::previewPath(const geometry_msgs::PoseArray pose_array)
{
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.header = pose_array.header;
    marker.ns = "preview";
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    double id = marker_array_.markers.size();

    for (unsigned int i = 0; i < pose_array.poses.size(); i++)
    {
        marker.id = id + i;
        marker.pose = pose_array.poses.at(i);
        marker_array_.markers.push_back(marker);
    }

    marker_pub_.publish(marker_array_);
}

void CartesianControllerUtils::adjustArrayLength(std::vector<cob_cartesian_controller::PathArray>& m)
{
    unsigned int max_steps = 0;
    for (unsigned int i = 0; i < m.size(); i++)
    {
        max_steps = std::max((unsigned int)m[i].array_.size(), max_steps);
    }

    for (unsigned int i = 0; i < m.size(); i++)
    {
        if (m[i].array_.size() < max_steps)
        {
            m[i].array_.resize(max_steps, m[i].array_.back());
        }
    }
}

void CartesianControllerUtils::copyMatrix(std::vector<double>* path_array, std::vector<cob_cartesian_controller::PathArray>& m)
{
    for (unsigned int i = 0; i < m.size(); i++)
    {
        path_array[i] = m[i].array_;
    }
}

double CartesianControllerUtils::roundUpToMultiplier(const double numberToRound, const double multiplier)
{
    return ( multiplier - std::fmod(numberToRound, multiplier) + numberToRound );
}
