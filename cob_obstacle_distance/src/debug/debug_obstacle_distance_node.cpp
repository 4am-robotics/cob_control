/*
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
 *   ROS package name: cob_obstacle_distance
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: May, 2015
 *
 * \brief
 *   Debug node visualizing the vector to the closest obstacle as well as the distance value through visualization_msgs/Marker.
 ****************************************************************/

#include <string.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "cob_obstacle_distance/ObstacleDistances.h"


class DebugObstacleDistance
{

ros::NodeHandle nh_;
ros::Publisher marker_pub_;
ros::Subscriber obstacle_distances_sub_;

std::string chain_base_link_;


public:

    int init()
    {
        if(!nh_.getParam("chain_base_link", this->chain_base_link_))
        {
            ROS_ERROR("Failed to get parameter \"chain_base_link\".");
            return -1;
        }
        
        marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("obstacle_distance/debug", 1, true);
        obstacle_distances_sub_ = this->nh_.subscribe("obstacle_distance", 1, &DebugObstacleDistance::obstacleDistancesCallback, this);

        return 0;
    }


    void obstacleDistancesCallback(const cob_obstacle_distance::ObstacleDistances::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray marker_array;
        
        for(unsigned int i=0; i<msg->distances.size(); i++)
        {
            //show distance vector as arrow
            visualization_msgs::Marker marker_vector;
            marker_vector.type = visualization_msgs::Marker::ARROW;
            marker_vector.lifetime = ros::Duration();
            marker_vector.action = visualization_msgs::Marker::ADD;
            marker_vector.ns = msg->distances[i].frame_of_interest;
            marker_vector.id = 42;
            marker_vector.header.frame_id = chain_base_link_;

            marker_vector.scale.x = 0.01;
            marker_vector.scale.y = 0.05;

            geometry_msgs::Point start;
            start.x = msg->distances[i].nearest_point_obstacle_vector.x;
            start.y = msg->distances[i].nearest_point_obstacle_vector.y;
            start.z = msg->distances[i].nearest_point_obstacle_vector.z;

            geometry_msgs::Point end;
            end.x = msg->distances[i].nearest_point_frame_vector.x;
            end.y = msg->distances[i].nearest_point_frame_vector.y;
            end.z = msg->distances[i].nearest_point_frame_vector.z;

            marker_vector.color.a = 1.0;
            marker_vector.color.g = 1.0;

            marker_vector.points.push_back(start);
            marker_vector.points.push_back(end);
            marker_array.markers.push_back(marker_vector);
            
            
            //show distance as text
            visualization_msgs::Marker marker_distance;
            marker_distance.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_distance.lifetime = ros::Duration();
            marker_distance.action = visualization_msgs::Marker::ADD;
            marker_distance.ns = msg->distances[i].frame_of_interest;
            marker_distance.id = 69;
            marker_distance.header.frame_id = msg->distances[i].header.frame_id;
            marker_distance.text = boost::lexical_cast<std::string>(boost::format("%.2f") % msg->distances[i].distance);
            
            marker_distance.scale.x = 0.1;
            marker_distance.scale.y = 0.1;
            marker_distance.scale.z = 0.1;
            
            marker_distance.color.a = 1.0;
            marker_distance.color.r = 1.0;
            marker_distance.color.g = 1.0;
            marker_distance.color.b = 1.0;
            
            marker_distance.pose.position.x = 0.15;
            
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

