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

#include "cob_obstacle_distance/ObstacleDistances.h"

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

#endif /* OBSTACLE_DISTANCE_DATA_TYPES_HPP_ */
