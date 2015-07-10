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

#include "cob_obstacle_distance/ObstacleDistances.h"

#define MESH_RES_MARKER 99u // Must be equal to "cob_twist_controller/cob_twist_cob.hpp"

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

struct ST_Frame2CollisionMesh
{
    std::unordered_map<std::string, std::string> m;

    ST_Frame2CollisionMesh()
    {
        m["arm_right_1_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_1_collision.stl";
        m["arm_left_1_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_1_collision.stl";

        m["arm_right_2_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_2_collision.stl";
        m["arm_left_2_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_2_collision.stl";

        m["arm_right_3_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_3_collision.stl";
        m["arm_left_3_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_3_collision.stl";

        m["arm_right_4_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_4_collision.stl";
        m["arm_left_4_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_4_collision.stl";

        m["arm_right_5_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_5_collision.stl";
        m["arm_left_5_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_5_collision.stl";

        m["arm_right_6_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_6_collision.stl";
        m["arm_left_6_link"] = "package://schunk_description/meshes/lwa4p_extended/arm_6_collision.stl";
    }
};

#endif /* OBSTACLE_DISTANCE_DATA_TYPES_HPP_ */
