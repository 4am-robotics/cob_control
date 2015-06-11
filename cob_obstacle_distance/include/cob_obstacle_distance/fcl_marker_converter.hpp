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
 *   Templates for the conversion between several fcl shapes and rviz markers
 *
 ****************************************************************/


#ifndef FCL_MARKER_CONVERTER_HPP_
#define FCL_MARKER_CONVERTER_HPP_

#include <boost/scoped_ptr.hpp>
#include "fcl/shape/geometric_shapes.h"

template <typename T>
class FclMarkerConverter
{
    private:
        FclMarkerConverter() {}
        FclMarkerConverter(T &t) {}
};

template<>
class FclMarkerConverter<fcl::Box>
{
    typedef boost::scoped_ptr<fcl::Box> sPtrBox;

    private:
        fcl::Box geo_shape_;

    public:
        FclMarkerConverter() : geo_shape_(fcl::Box(1.0, 1.0, 1.0)) {}
        FclMarkerConverter(fcl::Box &box) : geo_shape_(box) {}

        void assignValues(visualization_msgs::Marker &marker)
        {
            marker.scale.x = this->geo_shape_.side[0];
            marker.scale.y = this->geo_shape_.side[1];
            marker.scale.z = this->geo_shape_.side[2];
            marker.type = visualization_msgs::Marker::CUBE;
        }

        fcl::Box getGeoShape() const
        {
            return geo_shape_;
        }
};

template<>
class FclMarkerConverter<fcl::Sphere>
{
    typedef boost::scoped_ptr<fcl::Sphere> sPtrSphere;

    private:
        fcl::Sphere geo_shape_;

    public:
        FclMarkerConverter() : geo_shape_(fcl::Sphere(1.0)) {}
        FclMarkerConverter(fcl::Sphere &sphere) : geo_shape_(sphere) {}

        void assignValues(visualization_msgs::Marker &marker)
        {
            marker.scale.x = this->geo_shape_.radius;
            marker.scale.y = this->geo_shape_.radius;
            marker.scale.z = this->geo_shape_.radius;
            marker.type = visualization_msgs::Marker::SPHERE;
        }

        fcl::Sphere getGeoShape() const
        {
            return geo_shape_;
        }
};

template<>
class FclMarkerConverter<fcl::Cylinder>
{
    typedef boost::scoped_ptr<fcl::Cylinder> sPtrCylinder;

    private:
        fcl::Cylinder geo_shape_;

    public:
        FclMarkerConverter() : geo_shape_(fcl::Cylinder(1.0, 1.0)) {}
        FclMarkerConverter(fcl::Cylinder &cyl) : geo_shape_(cyl) {}

        void assignValues(visualization_msgs::Marker &marker)
        {
            marker.scale.x = this->geo_shape_.radius;
            marker.scale.y = this->geo_shape_.radius;
            marker.scale.z = this->geo_shape_.lz;
            marker.type = visualization_msgs::Marker::CYLINDER;
        }

        fcl::Cylinder getGeoShape() const
        {
            return geo_shape_;
        }
};


#endif /* FCL_MARKER_CONVERTER_HPP_ */
