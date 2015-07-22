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

#ifndef COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_HELPER_CLASSES_UTILS_H_
#define COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_HELPER_CLASSES_UTILS_H_

#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cob_cartesian_controller/helper_classes/data_structures.h>



class Utils{
    public:
        void showLevel(tf::Transform pos, std::string referenceFrame, int marker_id,double red, double green, double blue,std::string ns);
        void showDot(std::string referenceFrame, double x,double y,double z,int marker_id,double red, double green, double blue,std::string ns);
        void showMarker(tf::StampedTransform tf, std::string referenceFrame, int marker_id,double red, double green, double blue,std::string ns);
        bool epsilon_area(double x,double y, double z, double roll, double pitch, double yaw,double epsilon);
        geometry_msgs::Pose getEndeffectorPose(tf::TransformListener &listener, std::string referenceFrame, std::string chain_tip_link);
        void PoseToRPY(geometry_msgs::Pose pose, double &roll, double &pitch, double &yaw);
};

#endif /* COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_HELPER_CLASSES_UTILS_H_ */
