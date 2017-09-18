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


#ifndef MARKER_SHAPES_IMPL_HPP_
#define MARKER_SHAPES_IMPL_HPP_

#include "cob_obstacle_distance/marker_shapes/marker_shapes.hpp"

/* BEGIN MarkerShape ********************************************************************************************/
template <typename T>
MarkerShape<T>::MarkerShape(const std::string& root_frame, T& fcl_object,
      double x, double y, double z,
      double quat_x, double quat_y, double quat_z, double quat_w,
      double color_r, double color_g, double color_b, double color_a) : fcl_marker_converter_(fcl_object)
{
    this->init(root_frame, x, y, z, quat_x, quat_y, quat_z, quat_w, color_r, color_g, color_b, color_a);
}


template <typename T>
MarkerShape<T>::MarkerShape(const std::string& root_frame, double x, double y, double z,
            double quat_x, double quat_y, double quat_z, double quat_w,
            double color_r, double color_g, double color_b, double color_a)
{
    this->init(root_frame, x, y, z, quat_x, quat_y, quat_z, quat_w, color_r, color_g, color_b, color_a);
}

template <typename T>
geometry_msgs::Pose MarkerShape<T>::getMarkerPose() const
{
    return this->marker_.pose;
}


template <typename T>
geometry_msgs::Pose MarkerShape<T>::getOriginRelToFrame() const
{
    return this->origin_;
}


template <typename T>
void MarkerShape<T>::init(const std::string& root_frame, double x, double y, double z,
          double quat_x, double quat_y, double quat_z, double quat_w,
          double color_r, double color_g, double color_b, double color_a)
{
    marker_.pose.position.x = origin_.position.x = x;
    marker_.pose.position.y = origin_.position.y = y;
    marker_.pose.position.z = origin_.position.z = z;
    marker_.pose.orientation.x = origin_.orientation.x = quat_x;
    marker_.pose.orientation.y = origin_.orientation.y = quat_y;
    marker_.pose.orientation.z = origin_.orientation.z = quat_z;
    marker_.pose.orientation.w = origin_.orientation.w = quat_w;

    marker_.color.r = color_r;
    marker_.color.g = color_g;
    marker_.color.b = color_b;
    marker_.color.a = color_a;

    marker_.header.frame_id = root_frame;
    marker_.header.stamp = ros::Time::now();
    marker_.ns = g_marker_namespace;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.id = class_ctr_;

    marker_.lifetime = ros::Duration();

    fcl_marker_converter_.assignValues(marker_);

    BVH_RSS_t bvh;
    fcl_marker_converter_.getBvhModel(bvh);
    this->ptr_fcl_bvh_.reset(new BVH_RSS_t(bvh));
    this->ptr_fcl_bvh_->computeLocalAABB();
}


template <typename T>
inline uint32_t MarkerShape<T>::getId() const
{
    return this->marker_.id;
}


template <typename T>
inline void MarkerShape<T>::setColor(double color_r, double color_g, double color_b, double color_a)
{
    marker_.color.r = color_r;
    marker_.color.g = color_g;
    marker_.color.b = color_b;
    marker_.color.a = color_a;
    fcl_marker_converter_.assignValues(marker_);
}


template <typename T>
inline visualization_msgs::Marker MarkerShape<T>::getMarker()
{
    this->marker_.header.stamp = ros::Time::now();
    return this->marker_;
}


template <typename T>
fcl::CollisionObject MarkerShape<T>::getCollisionObject() const
{
    fcl::Transform3f x(fcl::Quaternion3f(this->marker_.pose.orientation.w,
                                         this->marker_.pose.orientation.x,
                                         this->marker_.pose.orientation.y,
                                         this->marker_.pose.orientation.z),
                       fcl::Vec3f(this->marker_.pose.position.x,
                                  this->marker_.pose.position.y,
                                  this->marker_.pose.position.z));
    fcl::CollisionObject cobj(this->ptr_fcl_bvh_, x);
    return cobj;
}


template <typename T>
inline void MarkerShape<T>::updatePose(const geometry_msgs::Vector3& pos, const geometry_msgs::Quaternion& quat)
{
    marker_.pose.position.x = pos.x;
    marker_.pose.position.y = pos.y;
    marker_.pose.position.z = pos.z;
    marker_.pose.orientation = quat;
}


template <typename T>
inline void MarkerShape<T>::updatePose(const geometry_msgs::Pose& pose)
{
    marker_.pose = pose;
}

/* END MarkerShape **********************************************************************************************/

#endif /* MARKER_SHAPES_IMPL_HPP_ */
