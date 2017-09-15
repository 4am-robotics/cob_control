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


#include <string>
#include <map>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "cob_control_msgs/ObstacleDistance.h"
#include "cob_control_msgs/ObstacleDistances.h"

class DebugObstacleDistance
{
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::Subscriber obstacle_distances_sub_;

    std::string chain_base_link_;

public:
    int init()
    {
        if (!nh_.getParam("chain_base_link", this->chain_base_link_))
        {
            ROS_ERROR("Failed to get parameter \"chain_base_link\".");
            return -1;
        }

        marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("obstacle_distance/distance_markers", 1, true);
        obstacle_distances_sub_ = this->nh_.subscribe("obstacle_distance", 1, &DebugObstacleDistance::obstacleDistancesCallback, this);

        return 0;
    }


    void obstacleDistancesCallback(const cob_control_msgs::ObstacleDistances::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray marker_array;
        std::map<std::string, cob_control_msgs::ObstacleDistance> relevant_obstacle_distances;

        for (uint32_t i = 0; i < msg->distances.size(); i++)
        {
            const std::string id = msg->distances[i].link_of_interest;
            if (relevant_obstacle_distances.count(id) > 0)
            {
                if (relevant_obstacle_distances[id].distance > msg->distances[i].distance)
                {
                    relevant_obstacle_distances[id] = msg->distances[i];
                }
            }
            else
            {
                relevant_obstacle_distances[id] = msg->distances[i];
            }
        }

        for (std::map<std::string, cob_control_msgs::ObstacleDistance>::const_iterator it = relevant_obstacle_distances.begin();
                it != relevant_obstacle_distances.end(); ++it)
        {
            // show distance vector as arrow
            visualization_msgs::Marker marker_vector;
            marker_vector.type = visualization_msgs::Marker::ARROW;
            marker_vector.lifetime = ros::Duration(0.5);
            marker_vector.action = visualization_msgs::Marker::ADD;
            marker_vector.ns = it->first;
            marker_vector.id = 42;
            marker_vector.header.frame_id = chain_base_link_;

            marker_vector.scale.x = 0.01;
            marker_vector.scale.y = 0.05;

            geometry_msgs::Point start;
            start.x = it->second.nearest_point_obstacle_vector.x;
            start.y = it->second.nearest_point_obstacle_vector.y;
            start.z = it->second.nearest_point_obstacle_vector.z;

            geometry_msgs::Point end;
            end.x = it->second.nearest_point_frame_vector.x;
            end.y = it->second.nearest_point_frame_vector.y;
            end.z = it->second.nearest_point_frame_vector.z;

            marker_vector.color.a = 1.0;
            marker_vector.color.g = 1.0;

            marker_vector.points.push_back(start);
            marker_vector.points.push_back(end);
            marker_array.markers.push_back(marker_vector);


            // show distance as text
            visualization_msgs::Marker marker_distance;
            marker_distance.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_distance.lifetime = ros::Duration(0.5);
            marker_distance.action = visualization_msgs::Marker::ADD;
            marker_distance.ns = it->first;
            marker_distance.id = 69;
            marker_distance.header.frame_id = chain_base_link_;
            marker_distance.text = boost::lexical_cast<std::string>(boost::format("%.3f") % it->second.distance);

            marker_distance.scale.x = 0.1;
            marker_distance.scale.y = 0.1;
            marker_distance.scale.z = 0.1;

            marker_distance.color.a = 1.0;
            // marker_distance.color.r = 1.0;
            // marker_distance.color.g = 1.0;
            // marker_distance.color.b = 1.0;
            marker_distance.color.r = 0.0;
            marker_distance.color.g = 0.0;
            marker_distance.color.b = 0.0;

            marker_distance.pose.position.x = it->second.nearest_point_frame_vector.x;
            marker_distance.pose.position.y = it->second.nearest_point_frame_vector.y + 0.05;
            marker_distance.pose.position.z = it->second.nearest_point_frame_vector.z;

            marker_array.markers.push_back(marker_distance);
        }

        this->marker_pub_.publish(marker_array);
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "debug_obstacle_distance_node");

    DebugObstacleDistance dod;
    if (dod.init() != 0)
    {
        ROS_ERROR("Failed to initialize DebugDistanceManager.");
        return -1;
    }

    ros::spin();
}

