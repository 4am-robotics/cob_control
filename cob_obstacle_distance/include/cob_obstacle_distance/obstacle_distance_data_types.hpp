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


#ifndef OBSTACLE_DISTANCE_DATA_TYPES_HPP_
#define OBSTACLE_DISTANCE_DATA_TYPES_HPP_

#include <ros/ros.h>
#include <stdint.h>
#include <unordered_map>
#include <shape_msgs/SolidPrimitive.h>
#include <visualization_msgs/Marker.h>

#define FCL_BOX_X 0u
#define FCL_BOX_Y 1u
#define FCL_BOX_Z 2u

#define FCL_RADIUS 0u
#define FCL_CYL_LENGTH 1u

#define MIN_DISTANCE 0.5 // [m]: filter for distances to be published!

#define DEFAULT_COL_ALPHA 0.6 // MoveIt! CollisionGeometry does not provide color -> Therefore use default value. 0.5 = Test for taking pictures -> robot arm should be visible behind obstacle

struct ShapeMsgTypeToVisMarkerType
{
    public:
        std::unordered_map<uint8_t, uint32_t> map_;
        std_msgs::ColorRGBA obstacle_color_;

        ShapeMsgTypeToVisMarkerType()
        {
            map_[shape_msgs::SolidPrimitive::BOX] = visualization_msgs::Marker::CUBE;
            map_[shape_msgs::SolidPrimitive::SPHERE] = visualization_msgs::Marker::SPHERE;
            map_[shape_msgs::SolidPrimitive::CYLINDER] = visualization_msgs::Marker::CYLINDER;

            obstacle_color_.r = 1.0;
            obstacle_color_.g = 0.0;
            obstacle_color_.b = 0.0;
            obstacle_color_.a = DEFAULT_COL_ALPHA;
        }
};


struct TriangleSupport
{
    fcl::Vec3f a;
    fcl::Vec3f b;
    fcl::Vec3f c;
};

static ShapeMsgTypeToVisMarkerType g_shapeMsgTypeToVisMarkerType;

#endif /* OBSTACLE_DISTANCE_DATA_TYPES_HPP_ */
