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


#pragma once
#ifndef COB_VELOCITY_LIMITED_MARKER_H
#define COB_VELOCITY_LIMITED_MARKER_H

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <visualization_msgs/Marker.h>

namespace cob_collision_velocity_filter
{

///
/// @class CollisionVelocityFilter
/// @brief checks for obstacles in driving direction and stops the robot
///
class VelocityLimitedMarker
{
public:
    ///
    /// @brief  Constructor
    ///
    VelocityLimitedMarker();

    ///
    /// @brief  Destructor
    ///
    ~VelocityLimitedMarker();

    ///
    /// @brief  Creates all directional markers, the method is called from the constructor.
    ///
    void createDirectionalMarkers();

    ///
    /// @brief  Creates all rotational markers, the method is called from the constructor.
    ///
    void createRotationalMarkers();

    ///
    /// @brief  Creates all the markers, the method is called from the constructor.
    ///
    void publishMarkers( double vel_x_desired,
                    double vel_x_actual,
                    double vel_y_desired,
                    double vel_y_actual,
                    double vel_theta_desired,
                    double vel_theta_actual);

    ///
    /// @brief  Calculates the color for the marker
    ////        based on the absolute difference of velocities.
    ///
    void interpolateColor(double velocity, std_msgs::ColorRGBA& color);

protected:
    // Velocity limited markers
    visualization_msgs::Marker x_pos_marker_, x_neg_marker_, y_pos_marker_, y_neg_marker_;
    visualization_msgs::Marker theta_pos_marker_, theta_neg_marker_;

    // a handle for this node
    ros::NodeHandle nh_;

    // Marker publisher
    ros::Publisher marker_pub_;

    // Is the marker disabled?
    bool disabled_;

    // Robot base frame
    std::string base_frame_;

    // Output topic name
    std::string topic_name_;

    // Marker lifetime
    double lifetime_;

    // Marker z-position
    double z_pos_;

    // last velocities
    double vx_last_, vy_last_, vtheta_last_;
};


}

#endif // COB_VELOCITY_LIMITED_MARKER_H

