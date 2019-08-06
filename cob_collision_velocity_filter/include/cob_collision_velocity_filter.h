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


#ifndef COB_COLLISION_VELOCITY_FILTER_H
#define COB_COLLISION_VELOCITY_FILTER_H

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <XmlRpc.h>

#include <pthread.h>

// ROS message includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

// ROS service includes
#include "cob_footprint_observer/GetFootprint.h"

// dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <cob_collision_velocity_filter/CollisionVelocityFilterConfig.h>

// BUT velocity limited marker
#include "velocity_limited_marker.h"

// Costmap for obstacle detection
#include <costmap_2d/costmap_2d_ros.h>

///
/// @class CollisionVelocityFilter
/// @brief checks for obstacles in driving direction and stops the robot
///
///
class CollisionVelocityFilter
{
public:

  ///
  /// @brief  Constructor
  ///
  CollisionVelocityFilter(costmap_2d::Costmap2DROS * costmap);

  ///
  /// @brief  Destructor
  ///
  ~CollisionVelocityFilter();

  ///
  /// @brief  reads twist command from teleop device (joystick, teleop_keyboard, ...) and calls functions
  ///         for collision check (obstacleHandler) and driving of the robot (performControllerStep)
  /// @param  twist - velocity command sent as twist message (twist.linear.x/y/z, twist.angular.x/y/z)
  ///
  void joystickVelocityCB(const geometry_msgs::Twist::ConstPtr &twist);

  ///
  /// @brief  reads obstacles from costmap
  /// @param  obstacles - 2D occupancy grid in rolling window mode!
  ///
  void readObstacles();

  ///
  /// @brief  Timer callback, calls GetFootprint Service and adjusts footprint
  ///
  void getFootprint(const ros::TimerEvent&);

  ///
  /// @brief  Dynamic reconfigure callback
  /// @param  config - configuration file with dynamic reconfigureable parameters
  /// @param  level - the result of ORing together all level values of the parameters that have changed, for now unnecessary
  ///
  void dynamicReconfigureCB(const cob_collision_velocity_filter::CollisionVelocityFilterConfig &config,
                            const uint32_t level);

  /// create a handle for this node, initialize node
  //public
  ros::NodeHandle nh_;

  //private
  ros::NodeHandle pnh_;

  /// Timer for periodically calling GetFootprint Service
  ros::Timer get_footprint_timer_;

  /// declaration of publisher
  ros::Publisher topic_pub_command_;
  ros::Publisher topic_pub_relevant_obstacles_;

  /// declaration of subscriber
  ros::Subscriber joystick_velocity_sub_, obstacles_sub_;

  /// dynamic reconfigure
  dynamic_reconfigure::Server<cob_collision_velocity_filter::CollisionVelocityFilterConfig> dyn_server_;
  dynamic_reconfigure::Server<cob_collision_velocity_filter::CollisionVelocityFilterConfig>::CallbackType dynCB_;

private:
  /* core functions */

  costmap_2d::Costmap2DROS* anti_collision_costmap_;
  //costmap_2d::Costmap2D costmap;
  ///
  /// @brief  checks distance to obstacles in driving direction and slows down/stops
  ///         robot and publishes command velocity to robot
  ///
  void performControllerStep();

  ///
  /// @brief  checks for obstacles in driving direction of the robot (rotation included)
  ///         and publishes relevant obstacles
  ///
  void obstacleHandler();

  /* helper functions */

  ///
  /// @brief  returns the sign of x
  ///
  double sign(double x);

  ///
  /// @brief  computes distance between two points
  /// @param  a,b - Points
  /// @return distance
  ///
  double getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b);

  ///
  /// @brief  checks if obstacle lies already within footprint -> this is ignored due to sensor readings of the hull etc
  /// @param  x_obstacle - x coordinate of obstacle in occupancy grid local costmap
  /// @param  y_obstacle - y coordinate of obstacle in occupancy grid local costmap
  /// @return true if obstacle outside of footprint
  ///
  bool obstacleValid(double x_obstacle, double y_obstacle);

  ///
  /// @brief  stops movement of the robot
  ///
  void stopMovement();

  pthread_mutex_t m_mutex;

  //obstacle_treshold
  int costmap_obstacle_treshold_;

  //frames
  std::string global_frame_, robot_frame_;

  //velocity
  geometry_msgs::Vector3 robot_twist_linear_, robot_twist_angular_;
  double v_max_, vtheta_max_;
  double ax_max_, ay_max_, atheta_max_;

  //obstacle avoidance
  std::vector<geometry_msgs::Point> robot_footprint_;
  double footprint_left_, footprint_right_, footprint_front_, footprint_rear_;
  double footprint_left_initial_, footprint_right_initial_, footprint_front_initial_, footprint_rear_initial_;
  bool costmap_received_;
  nav_msgs::OccupancyGrid last_costmap_received_, relevant_obstacles_;
  double influence_radius_, stop_threshold_, obstacle_damping_dist_, use_circumscribed_threshold_;
  double closest_obstacle_dist_, closest_obstacle_angle_;

  // variables for slow down behavior
  double last_time_;
  double kp_, kv_;
  double vx_last_, vy_last_, vtheta_last_;
  double virt_mass_;

  // BUT velocity limited marker
  cob_collision_velocity_filter::VelocityLimitedMarker velocity_limited_marker_;

};
//CollisionVelocityFilter

#endif

