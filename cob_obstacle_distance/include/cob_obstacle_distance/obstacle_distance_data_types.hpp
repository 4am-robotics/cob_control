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
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This header contains a definition for extra data types used in cob_obstacle_distance package.
 ****************************************************************/

#ifndef OBSTACLE_DISTANCE_DATA_TYPES_HPP_
#define OBSTACLE_DISTANCE_DATA_TYPES_HPP_

#include <ros/ros.h>
#include <Eigen/Core>
#include <stdint.h>
#include <unordered_map>

#include <shape_msgs/SolidPrimitive.h>
#include <visualization_msgs/Marker.h>

#include "cob_obstacle_distance/ObstacleDistances.h"

#define FCL_BOX_X 0u
#define FCL_BOX_Y 1u
#define FCL_BOX_Z 2u

#define FCL_RADIUS 0u
#define FCL_CYL_LENGTH 1u



struct ObstacleDistance
{
    ObstacleDistance(uint32_t shape_type) : min_distance(0.0), shape_type(shape_type)
    {}

    ObstacleDistance() : min_distance(0.0), shape_type(0)
    {}

    double min_distance;
    Eigen::Vector3d obstacle_pos;
    Eigen::Vector3d pos_on_manipulator;
    ros::Time timestamp;
    uint32_t shape_type;
};

struct ShapeMsgTypeToVisMarkerType
{
    public:
        std::unordered_map<uint8_t, uint32_t> map_;

        ShapeMsgTypeToVisMarkerType()
        {
            map_[shape_msgs::SolidPrimitive::BOX] = visualization_msgs::Marker::CUBE;
            map_[shape_msgs::SolidPrimitive::SPHERE] = visualization_msgs::Marker::SPHERE;
            map_[shape_msgs::SolidPrimitive::CYLINDER] = visualization_msgs::Marker::CYLINDER;
        }
};

static ShapeMsgTypeToVisMarkerType g_shapeMsgTypeToVisMarkerType;

#endif /* OBSTACLE_DISTANCE_DATA_TYPES_HPP_ */
