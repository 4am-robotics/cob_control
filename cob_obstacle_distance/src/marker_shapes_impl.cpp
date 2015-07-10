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
 *   Fully specialized implementation for the special case
 *   of a BVHModel MarkerShape!
 *
 ****************************************************************/

#include "cob_obstacle_distance/marker_shapes.hpp"
#include "cob_obstacle_distance/parsers/mesh_parser.hpp"

/* BEGIN MarkerShape ********************************************************************************************/
MarkerShape<BVH_RSS_t>::MarkerShape(const std::string root_frame, const std::string mesh_resource,
      double x, double y, double z,
      double quat_x, double quat_y, double quat_z, double quat_w,
      double color_r, double color_g, double color_b, double color_a)
      : is_drawn_(false)
{
    this->init(mesh_resource, root_frame, x, y, z, quat_x, quat_y, quat_z, quat_w, color_r, color_g, color_b, color_a);
}


void MarkerShape<BVH_RSS_t>::init(const std::string mesh_resource, const std::string root_frame, double x, double y, double z,
          double quat_x, double quat_y, double quat_z, double quat_w,
          double color_r, double color_g, double color_b, double color_a)
{
    MeshParser sp(mesh_resource);
    if(0 != sp.createBVH(this->fcl_bvh_))
    {
        ROS_ERROR("Could not create BVH model!");
    }

    marker_.pose.position.x = x;
    marker_.pose.position.y = y;
    marker_.pose.position.z = z;
    marker_.pose.orientation.x = quat_x;
    marker_.pose.orientation.y = quat_y;
    marker_.pose.orientation.z = quat_z;
    marker_.pose.orientation.w = quat_w;

    marker_.color.r = color_r;
    marker_.color.g = color_g;
    marker_.color.b = color_b;
    marker_.color.a = color_a;

    marker_.scale.x = 1.0;
    marker_.scale.y = 1.0;
    marker_.scale.z = 1.0;
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;

    marker_.header.frame_id = root_frame;
    marker_.header.stamp = ros::Time::now();
    marker_.ns = g_marker_namespace;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.id = class_ctr_;
    marker_.mesh_resource = mesh_resource;

    marker_.lifetime = ros::Duration();
}


inline uint32_t MarkerShape<BVH_RSS_t>::getId() const
{
    return this->marker_.id;
}


inline void MarkerShape<BVH_RSS_t>::setColor(double color_r, double color_g, double color_b, double color_a)
{
    marker_.color.r = color_r;
    marker_.color.g = color_g;
    marker_.color.b = color_b;
    marker_.color.a = color_a;
}


inline void MarkerShape<BVH_RSS_t>::updatePose(geometry_msgs::Vector3 &pos, geometry_msgs::Quaternion &quat)
{
    marker_.pose.position.x = pos.x;
    marker_.pose.position.y = pos.y;
    marker_.pose.position.z = pos.z;
    marker_.pose.orientation.x = quat.x;
    marker_.pose.orientation.y = quat.y;
    marker_.pose.orientation.z = quat.z;
    marker_.pose.orientation.w = quat.w;
}


inline visualization_msgs::Marker MarkerShape<BVH_RSS_t>::getMarker()
{
    this->marker_.header.stamp = ros::Time::now();
    return this->marker_;
}


inline void MarkerShape<BVH_RSS_t>::setDrawn()
{
    this->is_drawn_ = true;
}


inline bool MarkerShape<BVH_RSS_t>::isDrawn() const
{
    return this->is_drawn_;
}


fcl::CollisionObject MarkerShape<BVH_RSS_t>::getCollisionObject() const
{
    fcl::Transform3f x(fcl::Quaternion3f(this->marker_.pose.orientation.w,
                                         this->marker_.pose.orientation.x,
                                         this->marker_.pose.orientation.y,
                                         this->marker_.pose.orientation.z),
                       fcl::Vec3f(this->marker_.pose.position.x,
                                  this->marker_.pose.position.y,
                                  this->marker_.pose.position.z));

//    BVH_RSS_tgeoShape = fcl_marker_converter_.getGeoShape();
//    geoShape.computeLocalAABB();
    fcl::CollisionObject cobj(boost::shared_ptr<fcl::CollisionGeometry>(new BVH_RSS_t(fcl_bvh_)), x);
    return cobj;
}

/* END MarkerShape **********************************************************************************************/
