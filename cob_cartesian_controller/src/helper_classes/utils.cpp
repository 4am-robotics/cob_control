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

#include <cob_cartesian_controller/helper_classes/utils.h>


geometry_msgs::Pose CartesianControllerUtils::getEndeffectorPose(tf::TransformListener& listener, std::string reference_frame, std::string chain_tip_link)
{
    geometry_msgs::Pose pos;
    tf::StampedTransform stamped_transform;
    bool transformed = false;

    do
    {
        try
        {
            listener.lookupTransform(reference_frame, chain_tip_link, ros::Time(0), stamped_transform);
            transformed = true;
        }
        catch (tf::TransformException& ex)
        {
            transformed = false;
            ros::Duration(0.1).sleep();
        }
    }while(!transformed);

    pos.position.x = stamped_transform.getOrigin().x();
    pos.position.y = stamped_transform.getOrigin().y();
    pos.position.z = stamped_transform.getOrigin().z();
    pos.orientation.x = stamped_transform.getRotation()[0];
    pos.orientation.y = stamped_transform.getRotation()[1];
    pos.orientation.z = stamped_transform.getRotation()[2];
    pos.orientation.w = stamped_transform.getRotation()[3];

    return pos;
}

// Checks if the endeffector is in the area of the target frame
bool CartesianControllerUtils::epsilonArea(double x, double y, double z, double roll, double pitch, double yaw, double epsilon)
{
    bool x_okay, y_okay, z_okay = false;
    bool roll_okay, pitch_okay, yaw_okay = false;

    yaw_okay    = std::fabs(yaw)   < epsilon;
    pitch_okay  = std::fabs(pitch) < epsilon;
    roll_okay   = std::fabs(roll)  < epsilon;
    z_okay      = std::fabs(z)     < epsilon;
    y_okay      = std::fabs(y)     < epsilon;
    x_okay      = std::fabs(x)     < epsilon;

    if(x_okay && y_okay && z_okay && roll_okay && pitch_okay && yaw_okay)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void CartesianControllerUtils::showMarker(tf::StampedTransform tf, std::string reference_frame, int marker_id, double red, double green, double blue, std::string ns)
{
    //ToDo: vis_pub should be a member
    //ToDo: where is the advertise for vis_pub?
    ros::Publisher vis_pub;
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = reference_frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = tf.getOrigin().x();
    marker.pose.position.y = tf.getOrigin().y();
    marker.pose.position.z = tf.getOrigin().z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0;
    
    vis_pub.publish( marker );
}

void CartesianControllerUtils::showDot(std::string reference_frame, double x, double y, double z, int marker_id, double red, double green, double blue, std::string ns)
{
    //ToDo: vis_pub should be a member
    //ToDo: where is the advertise for vis_pub?
    ros::Publisher vis_pub;
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = reference_frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0;

    vis_pub.publish( marker );
}

void CartesianControllerUtils::showLevel(tf::Transform pos, std::string reference_frame, int marker_id, double red, double green, double blue, std::string ns)
{
    //ToDo: vis_pub should be a member
    //ToDo: where is the advertise for vis_pub?
    ros::Publisher vis_pub;
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = reference_frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos.getOrigin().x();
    marker.pose.position.y = pos.getOrigin().y();
    marker.pose.position.z = pos.getOrigin().z();
    marker.pose.orientation.x = pos.getRotation()[0];
    marker.pose.orientation.y = pos.getRotation()[1];
    marker.pose.orientation.z = pos.getRotation()[2];
    marker.pose.orientation.w = pos.getRotation()[3];
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0.01;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 0.2;

    vis_pub.publish( marker );
}

void CartesianControllerUtils::poseToRPY(geometry_msgs::Pose pose, double& roll, double& pitch, double& yaw)
{
    tf::Quaternion q = tf::Quaternion(pose.orientation.x,
                                      pose.orientation.y,
                                      pose.orientation.z,
                                      pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
}
