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


#ifndef COB_FOOTPRINT_OBSERVER_H
#define COB_FOOTPRINT_OBSERVER_H

//##################
//#### includes ####

// ROS includes
#include <ros/ros.h>
#include <XmlRpc.h>

// message includes
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_listener.h>

#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <cob_footprint_observer/GetFootprint.h>
///
/// @class FootprintObserver
/// @brief checks the footprint of care-o-bot and advertises a service to get the adjusted footprint
///
///
class FootprintObserver
{
  public:
    ///
    /// @brief  constructor
    ///
    FootprintObserver();
    ///
    /// @brief  destructor
    ///
    ~FootprintObserver();

    ///
    /// @brief  checks the footprint of the robot if it needs to be enlarged due to arm or tray
    ///
    void checkFootprint();

    ///
    /// @brief  callback for GetFootprint service
    /// @param  req - request message to service
    /// @param  resp - response message from service
    /// @return service call successfull
    ///
    bool getFootprintCB(cob_footprint_observer::GetFootprint::Request &req, cob_footprint_observer::GetFootprint::Response &resp);

    // public members
    ros::NodeHandle nh_;
    ros::Publisher topic_pub_footprint_;
    ros::ServiceServer srv_get_footprint_;

  private:
    ///
    /// @brief  loads the robot footprint from the costmap node
    /// @param  node - costmap node to check for footprint parameter
    /// @return points of a polygon specifying the footprint
    ///
    std::vector<geometry_msgs::Point> loadRobotFootprint(ros::NodeHandle node);

    ///
    /// @brief  publishes the adjusted footprint as geometry_msgs::StampedPolygon message
    ///
    void publishFootprint();

    ///
    /// @brief  computes the sign of x
    /// @param  x - number
    /// @return sign of x
    ///
    double sign(double x);

    // private members
    std::vector<geometry_msgs::Point> robot_footprint_;
    double epsilon_;
    double footprint_front_initial_, footprint_rear_initial_, footprint_left_initial_, footprint_right_initial_;
    double footprint_front_, footprint_rear_, footprint_left_, footprint_right_;
    tf::TransformListener tf_listener_;
    std::string frames_to_check_;
    std::string robot_base_frame_;

    pthread_mutex_t m_mutex;

    ros::Time last_tf_missing_;
    unsigned int times_warned_;
};

#endif
