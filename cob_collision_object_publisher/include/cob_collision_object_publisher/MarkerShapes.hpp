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
 *   ROS package name: cob_collision_object_publisher
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: May, 2015
 *
 * \brief
 *   This header contains the description and template implementation of MarkerShapes
 *   which represent a combination of ROS markers and FCL geometric shapes.
 *
 ****************************************************************/

#ifndef MARKERSHAPES_HPP_
#define MARKERSHAPES_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "fcl/shape/geometric_shapes.h"
#include "fcl/collision_object.h"

#include "cob_collision_object_publisher/fcl_marker_converter.hpp"

static const std::string g_marker_namespace = "collision_object";
static const std::string g_frame_id = "base_link";

class IMarkerShape
{
    protected:
        static uint32_t classCtr_;

    public:
         IMarkerShape()
         {
             classCtr_++;
         }

         virtual uint32_t getId() const = 0;
         virtual void setColor(double color_r, double color_g, double color_b, double color_a = 1.0) = 0;
         virtual visualization_msgs::Marker getMarker() = 0;
         virtual void setDrawn() = 0;
         virtual bool isDrawn() const = 0;
         virtual fcl::CollisionObject getCollisionObject() const = 0;
         virtual ~IMarkerShape() {}
};

uint32_t IMarkerShape::classCtr_ = 0;

/* BEGIN MarkerShape ****************************************************************************************/
/// Template class implementation for box, sphere and cylinder fcl::shapes. Creates visualization marker.
template
<typename T>
class MarkerShape : public IMarkerShape
{
    private:
        bool isDrawn_;
        FclMarkerConverter<T> fclMarkerConverter_;
        visualization_msgs::Marker marker_;

        void init(double x, double y, double z,
                  double quat_x, double quat_y, double quat_z, double quat_w,
                  double color_r, double color_g, double color_b, double color_a)
        {
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

            marker_.header.frame_id = g_frame_id;
            marker_.header.stamp = ros::Time::now();
            marker_.ns = g_marker_namespace;
            marker_.action = visualization_msgs::Marker::ADD;
            marker_.id = classCtr_;

            fclMarkerConverter_.assignValues(marker_);
        }

    public:
        MarkerShape(T &fclObject, geometry_msgs::Pose &pose, std_msgs::ColorRGBA &col)
        : MarkerShape(fclObject,
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                col.r, col.g, col.b, col.a)
        {
        }

        MarkerShape(T &fclObject, geometry_msgs::Point &pos, geometry_msgs::Quaternion &quat, std_msgs::ColorRGBA &col)
        : MarkerShape(fclObject, pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w, col.r, col.g, col.b, col.a)
        {
        }

        MarkerShape(T &fclObject,
              double x, double y, double z,
              double quat_x = 0.0, double quat_y = 0.0, double quat_z = 0.0, double quat_w = 1.0,
              double color_r = 0.0, double color_g = 0.0, double color_b = 0.0, double color_a = 1.0)
              : fclMarkerConverter_(fclObject),
                isDrawn_(false)
        {
            this->init(x, y, z, quat_x, quat_y, quat_z, quat_w, color_r, color_g, color_b, color_a);
        }

        MarkerShape(double x, double y, double z,
                    double quat_x = 0.0, double quat_y = 0.0, double quat_z = 0.0, double quat_w = 1.0,
                    double color_r = 0.0, double color_g = 0.0, double color_b = 0.0, double color_a = 1.0) : isDrawn_(false)
        {
            this->init(x, y, z, quat_x, quat_y, quat_z, quat_w, color_r, color_g, color_b, color_a);
        }

        inline uint32_t getId() const
        {
            return this->marker_.id;
        }

        inline void setColor(double color_r, double color_g, double color_b, double color_a = 1.0)
        {
            marker_.color.r = color_r;
            marker_.color.g = color_g;
            marker_.color.b = color_b;
            marker_.color.a = color_a;
        }

        inline visualization_msgs::Marker getMarker()
        {
            this->marker_.header.stamp = ros::Time::now();
            return this->marker_;
        }

        inline void setDrawn()
        {
            this->isDrawn_ = true;
        }

        inline bool isDrawn() const
        {
            return this->isDrawn_;
        }

        fcl::CollisionObject getCollisionObject() const
        {
            fcl::Transform3f x(fcl::Quaternion3f(this->marker_.pose.orientation.w,
                                                 this->marker_.pose.orientation.x,
                                                 this->marker_.pose.orientation.y,
                                                 this->marker_.pose.orientation.z),
                               fcl::Vec3f(this->marker_.pose.position.x,
                                          this->marker_.pose.position.y,
                                          this->marker_.pose.position.z));

            T *ptrGeoShape = fclMarkerConverter_.getGeoShape();
            fcl::CollisionObject cobj(boost::shared_ptr<fcl::CollisionGeometry>(ptrGeoShape), x);

    //            fcl::Quaternion3f q = cobj.getQuatRotation();
    //            fcl::Vec3f v = cobj.getTranslation();
    //            ROS_INFO_STREAM("Quaternion3f: " << q.getW() << "; " << q.getX() << "; " << q.getY() << "; " << q.getZ() << std::endl);
    //            ROS_INFO_STREAM("Vec3f: " << v << std::endl);

            return cobj;
        }

        virtual ~MarkerShape(){}
};
/* END MarkerShape **********************************************************************************************/

#endif /* MARKERSHAPES_HPP_ */
