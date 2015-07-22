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


geometry_msgs::Pose Utils::getEndeffectorPose(tf::TransformListener &listener, std::string referenceFrame, std::string chain_tip_link)
{
    geometry_msgs::Pose pos;
    tf::StampedTransform stampedTransform;
    bool transformed = false;

    do{
        // Get transformation
        try{
            listener.lookupTransform(referenceFrame, chain_tip_link, ros::Time(0), stampedTransform);
            transformed = true;
        }
        catch (tf::TransformException &ex) {
            transformed = false;
            ros::Duration(0.1).sleep();
        }
    }while(!transformed);

    pos.position.x=stampedTransform.getOrigin().x();
    pos.position.y=stampedTransform.getOrigin().y();
    pos.position.z=stampedTransform.getOrigin().z();
    pos.orientation.x = stampedTransform.getRotation()[0];
    pos.orientation.y = stampedTransform.getRotation()[1];
    pos.orientation.z = stampedTransform.getRotation()[2];
    pos.orientation.w = stampedTransform.getRotation()[3];

    return pos;
}

// Checks if the endeffector is in the area of the 'br' frame
bool Utils::epsilon_area(double x,double y, double z, double roll, double pitch, double yaw, double epsilon)
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

void Utils::showMarker(tf::StampedTransform tf, std::string referenceFrame, int marker_id, double red, double green, double blue, std::string ns)
{
    ros::Publisher vis_pub;
    visualization_msgs::Marker marker;
    marker.header.frame_id = referenceFrame;
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

void Utils::showDot(std::string referenceFrame, double x, double y, double z, int marker_id, double red, double green, double blue, std::string ns)
{
    ros::Publisher vis_pub;
    visualization_msgs::Marker marker;
    marker.header.frame_id = referenceFrame;
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

void Utils::showLevel(tf::Transform pos, std::string referenceFrame, int marker_id, double red, double green, double blue,std::string ns)
{
    ros::Publisher vis_pub;
    visualization_msgs::Marker marker;
    marker.header.frame_id = referenceFrame;
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

void Utils::PoseToRPY(geometry_msgs::Pose pose, double &roll, double &pitch, double &yaw)
{
    tf::Quaternion q = tf::Quaternion(pose.orientation.x,
                                      pose.orientation.y,
                                      pose.orientation.z,
                                      pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
}
