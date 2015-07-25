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

#ifndef COB_CARTESIAN_CONTROLLER_UTILS_H_
#define COB_CARTESIAN_CONTROLLER_UTILS_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>


class CartesianControllerUtils
{
public:
    tf::StampedTransform getStampedTransform(std::string target_frame, std::string source_frame);
    geometry_msgs::Pose getPose(std::string target_frame, std::string source_frame);
    
    bool inEpsilonArea(tf::StampedTransform stamped_transform, double epsilon);
    void poseToRPY(geometry_msgs::Pose pose, double& roll, double& pitch, double& yaw);
    
    //Note: 
    //  all functions related to plotting the Cartesian trajectory 
    //  by publishing visualization_msgs/Marker to a topic
    //  have been removed in favor of debug_trajectory_marker_node (cob_twist_controller)
    //
    //Example Usage:
    //  <node ns="arm" name="debug_trajectory_marker_node" pkg="cob_twist_controller" type="debug_trajectory_marker_node" cwd="node" respawn="false" output="screen">
    //    <param name="target_frame" type="str" value="cartesian_target" />
    //  </node>


private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
};

#endif /* COB_CARTESIAN_CONTROLLER_UTILS_H_ */
