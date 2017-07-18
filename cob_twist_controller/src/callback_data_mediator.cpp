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


#include <limits>
#include <ros/ros.h>

#include "cob_twist_controller/callback_data_mediator.h"

#include <eigen_conversions/eigen_msg.h>

/// Counts all currently available distances to obstacles.
uint32_t CallbackDataMediator::obstacleDistancesCnt()
{
    boost::mutex::scoped_lock lock(distances_to_obstacles_lock_);
    return this->obstacle_distances_.size();
}

/// Consumer: Consumes elements from distances container
bool CallbackDataMediator::fill(ConstraintParamsCA& params_ca)
{
    boost::mutex::scoped_lock lock(distances_to_obstacles_lock_);
    bool success = false;
    double last_min_distance = std::numeric_limits<double>::max();
    params_ca.current_distances_.clear();
    for (ObstacleDistancesIter_t it = this->obstacle_distances_.begin(); it != this->obstacle_distances_.end(); it++)
    {
        if (it->first == params_ca.id_)  // select the appropriate distances for frame id of interest
        {
            params_ca.current_distances_ = it->second;  // copy all distances for frame to current distances of param struct
            success = true;
        }
    }

    return success;
}

/// Can be used to fill parameters for joint limit avoidance.
bool CallbackDataMediator::fill(ConstraintParamsJLA& params_jla)
{
    return true;
}

/// Producer: Fills obstacle distances but only when they are empty
void CallbackDataMediator::distancesToObstaclesCallback(const cob_control_msgs::ObstacleDistances::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(distances_to_obstacles_lock_);
    this->obstacle_distances_.clear();
    for (cob_control_msgs::ObstacleDistances::_distances_type::const_iterator it = msg->distances.begin(); it != msg->distances.end(); it++)
    {
        ObstacleDistanceData d;
        d.min_distance = it->distance;
        tf::vectorMsgToEigen(it->frame_vector, d.frame_vector);
        tf::vectorMsgToEigen(it->nearest_point_frame_vector, d.nearest_point_frame_vector);
        tf::vectorMsgToEigen(it->nearest_point_obstacle_vector, d.nearest_point_obstacle_vector);
        this->obstacle_distances_[it->link_of_interest].push_back(d);
    }
}
