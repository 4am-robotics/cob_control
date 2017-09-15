/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


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
