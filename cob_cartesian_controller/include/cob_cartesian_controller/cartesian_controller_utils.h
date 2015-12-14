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

#ifndef COB_CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_UTILS_H
#define COB_CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_UTILS_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_cartesian_controller/cartesian_controller_data_types.h>

class CartesianControllerUtils
{
public:
    CartesianControllerUtils()
    {
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("cartesian_controller/preview_path", 1);
    }

    void transformPose(const std::string source_frame, const std::string target_frame, const geometry_msgs::Pose pose_in, geometry_msgs::Pose& pose_out);
    tf::StampedTransform getStampedTransform(const std::string& target_frame, const std::string& source_frame);
    geometry_msgs::Pose getPose(const std::string& target_frame, const std::string& source_frame);

    bool inEpsilonArea(const tf::StampedTransform& stamped_transform, const double epsilon);
    void poseToRPY(const geometry_msgs::Pose& pose, double& roll, double& pitch, double& yaw);

    void previewPath(const geometry_msgs::PoseArray pose_array);

    void adjustArrayLength(std::vector<cob_cartesian_controller::PathArray>& m);
    void copyMatrix(std::vector<double>* path_array, std::vector<cob_cartesian_controller::PathArray>& m);
    double roundUpToMultiplier(const double numberToRound, const double multiplier);

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    visualization_msgs::MarkerArray marker_array_;

    ros::Publisher marker_pub_;
};

#endif  // COB_CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_UTILS_H
