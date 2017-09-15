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


#ifndef MARKER_SHAPES_HPP_
#define MARKER_SHAPES_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>

#include "fcl/shape/geometric_shapes.h"
#include "fcl/collision_object.h"

#include "cob_obstacle_distance/fcl_marker_converter.hpp"
#include "cob_obstacle_distance/marker_shapes/marker_shapes_interface.hpp"

#include <fcl/distance.h>
#include <fcl/collision_data.h>

static const std::string g_marker_namespace = "collision_object";

/* BEGIN MarkerShape ********************************************************************************************/
/// Template class implementation for box, sphere and cylinder fcl::shapes. Creates visualization marker.
template
<typename T>
class MarkerShape : public IMarkerShape
{
    private:
        FclMarkerConverter<T> fcl_marker_converter_;
        std::shared_ptr<BVH_RSS_t> ptr_fcl_bvh_;

        void init(const std::string& root_frame, double x, double y, double z,
                  double quat_x, double quat_y, double quat_z, double quat_w,
                  double color_r, double color_g, double color_b, double color_a);

    public:
        MarkerShape(const std::string& root_frame, T& fcl_object, const geometry_msgs::Pose& pose, const std_msgs::ColorRGBA& col)
        : MarkerShape(root_frame, fcl_object,
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                col.r, col.g, col.b, col.a)
        {
        }

        MarkerShape(const std::string& root_frame, T& fcl_object, const geometry_msgs::Point& pos, const geometry_msgs::Quaternion& quat, const std_msgs::ColorRGBA& col)
        : MarkerShape(root_frame, fcl_object, pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w, col.r, col.g, col.b, col.a)
        {
        }

        MarkerShape(const std::string& root_frame, T& fcl_object,
              double x, double y, double z,
              double quat_x = 0.0, double quat_y = 0.0, double quat_z = 0.0, double quat_w = 1.0,
              double color_r = 0.0, double color_g = 0.0, double color_b = 0.0, double color_a = 1.0);

        MarkerShape(const std::string& root_frame, double x, double y, double z,
                    double quat_x = 0.0, double quat_y = 0.0, double quat_z = 0.0, double quat_w = 1.0,
                    double color_r = 0.0, double color_g = 0.0, double color_b = 0.0, double color_a = 1.0);

        inline geometry_msgs::Pose getMarkerPose() const;

        inline geometry_msgs::Pose getOriginRelToFrame() const;

        /**
         * @param Returns the marker id with that it is published to RVIZ.
         */
        inline uint32_t getId() const;

        inline void setColor(double color_r, double color_g, double color_b, double color_a = 1.0);

        /**
         * @return Gets the visualization marker of this MarkerShape.
         */
        inline visualization_msgs::Marker getMarker();

        inline void updatePose(const geometry_msgs::Vector3& pos, const geometry_msgs::Quaternion& quat);

        inline void updatePose(const geometry_msgs::Pose& pose);

        /**
         * @return A fcl::CollisionObject to calculate distances to other objects or check whether collision occurred or not.
         */
        fcl::CollisionObject getCollisionObject() const;

        virtual ~MarkerShape(){}
};
/* END MarkerShape **********************************************************************************************/



/* BEGIN MarkerShape ********************************************************************************************/
template <>
class MarkerShape<BVH_RSS_t> : public IMarkerShape
{
    private:
        std::shared_ptr<BVH_RSS_t> ptr_fcl_bvh_;

        void init(const std::string& root_frame, const std::string& mesh_resource, double x, double y, double z,
                  double quat_x, double quat_y, double quat_z, double quat_w,
                  double color_r, double color_g, double color_b, double color_a);

    public:
        MarkerShape(const std::string& root_frame, const shape_msgs::Mesh& mesh, const geometry_msgs::Pose& pose, const std_msgs::ColorRGBA& col);

        MarkerShape(const std::string& root_frame, const std::string& mesh_resource, const geometry_msgs::Pose& pose, const std_msgs::ColorRGBA& col)
        : MarkerShape(root_frame, mesh_resource,
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                col.r, col.g, col.b, col.a)
        {
        }

        MarkerShape(const std::string& root_frame, const std::string& mesh_resource, const geometry_msgs::Point& pos, const geometry_msgs::Quaternion& quat, const std_msgs::ColorRGBA& col)
        : MarkerShape(root_frame, mesh_resource, pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w, col.r, col.g, col.b, col.a)
        {
        }

        MarkerShape(const std::string& root_frame, const std::string& mesh_resource,
              double x, double y, double z,
              double quat_x = 0.0, double quat_y = 0.0, double quat_z = 0.0, double quat_w = 1.0,
              double color_r = 0.0, double color_g = 0.0, double color_b = 0.0, double color_a = 1.0);

        inline geometry_msgs::Pose getMarkerPose() const;

        inline geometry_msgs::Pose getOriginRelToFrame() const;

        /**
         * @param Returns the marker id with that it is published to RVIZ.
         */
        inline uint32_t getId() const;

        inline void setColor(double color_r, double color_g, double color_b, double color_a = 1.0);

        /**
         * @return Gets the visualization marker of this MarkerShape.
         */
        inline visualization_msgs::Marker getMarker();

        inline void updatePose(const geometry_msgs::Vector3& pos, const geometry_msgs::Quaternion& quat);

        inline void updatePose(const geometry_msgs::Pose& pose);

        /**
         * @return A fcl::CollisionObject to calculate distances to other objects or check whether collision occurred or not.
         */
        fcl::CollisionObject getCollisionObject() const;

        virtual ~MarkerShape(){}
};
/* END MarkerShape **********************************************************************************************/




#include "cob_obstacle_distance/marker_shapes/marker_shapes_impl.hpp"

#endif /* MARKER_SHAPES_HPP_ */
