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

#include "cob_obstacle_distance/marker_shapes/marker_shapes.hpp"
#include "cob_obstacle_distance/parsers/mesh_parser.hpp"

/* BEGIN MarkerShape ********************************************************************************************/
MarkerShape<BVH_RSS_t>::MarkerShape(const std::string& root_frame,
                                    const shape_msgs::Mesh& mesh,
                                    const geometry_msgs::Pose& pose,
                                    const std_msgs::ColorRGBA& col)
                                    : is_drawn_(false)
{
    this->fcl_bvh_.beginModel();
    for(shape_msgs::MeshTriangle tri : mesh.triangles)
    {
        uint32_t v_idx_1 = tri.vertex_indices.elems[0];
        uint32_t v_idx_2 = tri.vertex_indices.elems[1];
        uint32_t v_idx_3 = tri.vertex_indices.elems[2];

        fcl::Vec3f v1(mesh.vertices[v_idx_1].x, mesh.vertices[v_idx_1].y, mesh.vertices[v_idx_1].z);
        fcl::Vec3f v2(mesh.vertices[v_idx_2].x, mesh.vertices[v_idx_2].y, mesh.vertices[v_idx_2].z);
        fcl::Vec3f v3(mesh.vertices[v_idx_3].x, mesh.vertices[v_idx_3].y, mesh.vertices[v_idx_3].z);

        this->fcl_bvh_.addTriangle(v1, v2, v3);
    }

    this->fcl_bvh_.endModel();
    this->fcl_bvh_.computeLocalAABB();

    marker_.pose = pose;
    marker_.color = col;

    marker_.scale.x = 1.0;
    marker_.scale.y = 1.0;
    marker_.scale.z = 1.0;
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;

    marker_.header.frame_id = root_frame;
    marker_.header.stamp = ros::Time::now();
    marker_.ns = g_marker_namespace;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.id = IMarkerShape::class_ctr_;
    marker_.mesh_resource = ""; // TODO: Not given in this case: can happen e.g. when moveit_msgs/CollisionObject was given!

    marker_.lifetime = ros::Duration();
}


MarkerShape<BVH_RSS_t>::MarkerShape(const std::string& root_frame, const std::string& mesh_resource,
      double x, double y, double z,
      double quat_x, double quat_y, double quat_z, double quat_w,
      double color_r, double color_g, double color_b, double color_a)
      : is_drawn_(false)
{
    this->init(mesh_resource, root_frame, x, y, z, quat_x, quat_y, quat_z, quat_w, color_r, color_g, color_b, color_a);
}


void MarkerShape<BVH_RSS_t>::init(const std::string& mesh_resource, const std::string& root_frame, double x, double y, double z,
          double quat_x, double quat_y, double quat_z, double quat_w,
          double color_r, double color_g, double color_b, double color_a)
{
    MeshParser sp(mesh_resource);
    if(0 != sp.createBVH(this->fcl_bvh_))
    {
        ROS_ERROR("Could not create BVH model!");
    }

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

    marker_.scale.x = 1.0;
    marker_.scale.y = 1.0;
    marker_.scale.z = 1.0;
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;

    marker_.header.frame_id = root_frame;
    marker_.header.stamp = ros::Time::now();
    marker_.ns = g_marker_namespace;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.id = IMarkerShape::class_ctr_;
    marker_.mesh_resource = mesh_resource;

    marker_.lifetime = ros::Duration();
}


inline geometry_msgs::Pose MarkerShape<BVH_RSS_t>::getMarkerPose() const
{
    return this->marker_.pose;
}


inline geometry_msgs::Pose MarkerShape<BVH_RSS_t>::getOriginRelToFrame() const
{
    return this->origin_;
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


inline void MarkerShape<BVH_RSS_t>::updatePose(const geometry_msgs::Vector3& pos, const geometry_msgs::Quaternion& quat)
{
    marker_.pose.position.x = pos.x;
    marker_.pose.position.y = pos.y;
    marker_.pose.position.z = pos.z;
    marker_.pose.orientation = quat;
}


inline void MarkerShape<BVH_RSS_t>::updatePose(const geometry_msgs::Pose& pose)
{
    marker_.pose = pose;
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
    fcl::CollisionObject cobj(boost::shared_ptr<fcl::CollisionGeometry>(new BVH_RSS_t(fcl_bvh_)), x);
    return cobj;
}

/* END MarkerShape **********************************************************************************************/
