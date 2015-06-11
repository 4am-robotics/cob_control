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
 *   This header contains the definition of MarkerShapes
 *   which represent a combination of ROS markers and FCL geometric shapes.
 *
 ****************************************************************/

#ifndef MARKER_SHAPES_HPP_
#define MARKER_SHAPES_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "fcl/shape/geometric_shapes.h"
#include "fcl/collision_object.h"

#include "cob_obstacle_distance/fcl_marker_converter.hpp"
#include "cob_obstacle_distance/marker_shapes_interface.hpp"

static const std::string g_marker_namespace = "collision_object";
static const std::string g_frame_id = "base_link";

/* BEGIN MarkerShape ********************************************************************************************/
/// Template class implementation for box, sphere and cylinder fcl::shapes. Creates visualization marker.
template
<typename T>
class MarkerShape : public IMarkerShape
{
    private:
        bool is_drawn_;
        FclMarkerConverter<T> fcl_marker_converter_;
        visualization_msgs::Marker marker_;

        void init(double x, double y, double z,
                  double quat_x, double quat_y, double quat_z, double quat_w,
                  double color_r, double color_g, double color_b, double color_a);

    public:
        MarkerShape(T &fcl_object, geometry_msgs::Pose &pose, std_msgs::ColorRGBA &col)
        : MarkerShape(fcl_object,
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                col.r, col.g, col.b, col.a)
        {
        }

        MarkerShape(T &fcl_object, geometry_msgs::Point &pos, geometry_msgs::Quaternion &quat, std_msgs::ColorRGBA &col)
        : MarkerShape(fcl_object, pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w, col.r, col.g, col.b, col.a)
        {
        }

        MarkerShape(T &fcl_object,
              double x, double y, double z,
              double quat_x = 0.0, double quat_y = 0.0, double quat_z = 0.0, double quat_w = 1.0,
              double color_r = 0.0, double color_g = 0.0, double color_b = 0.0, double color_a = 1.0);

        MarkerShape(double x, double y, double z,
                    double quat_x = 0.0, double quat_y = 0.0, double quat_z = 0.0, double quat_w = 1.0,
                    double color_r = 0.0, double color_g = 0.0, double color_b = 0.0, double color_a = 1.0);

        inline uint32_t getId() const;
        inline void setColor(double color_r, double color_g, double color_b, double color_a = 1.0);
        inline visualization_msgs::Marker getMarker();
        inline void setDrawn();
        inline bool isDrawn() const;
        fcl::CollisionObject getCollisionObject() const;

        virtual ~MarkerShape(){}
};
/* END MarkerShape **********************************************************************************************/

#include "cob_obstacle_distance/marker_shapes_impl.hpp"

#endif /* MARKER_SHAPES_HPP_ */
