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
        //sPtrBox geoShape_;
        fcl::Box geoShape_;

    public:
        FclMarkerConverter() : geoShape_(fcl::Box(1.0, 1.0, 1.0)) {}
        FclMarkerConverter(fcl::Box &box) : geoShape_(box) {}

        void assignValues(visualization_msgs::Marker &marker)
        {
            marker.scale.x = this->geoShape_.side[0];
            marker.scale.y = this->geoShape_.side[1];
            marker.scale.z = this->geoShape_.side[2];
            marker.type = visualization_msgs::Marker::CUBE;
        }

        fcl::Box getGeoShape() const
        {
            return geoShape_;
        }
};

template<>
class FclMarkerConverter<fcl::Sphere>
{
    typedef boost::scoped_ptr<fcl::Sphere> sPtrSphere;

    private:
        //sPtrSphere geoShape_;
        fcl::Sphere geoShape_;

    public:
        FclMarkerConverter() : geoShape_(fcl::Sphere(1.0)) {}
        FclMarkerConverter(fcl::Sphere &sphere) : geoShape_(sphere) {}

        void assignValues(visualization_msgs::Marker &marker)
        {
            marker.scale.x = this->geoShape_.radius;
            marker.scale.y = this->geoShape_.radius;
            marker.scale.z = this->geoShape_.radius;
            marker.type = visualization_msgs::Marker::SPHERE;
        }

        fcl::Sphere getGeoShape() const
        {
            return geoShape_;
        }
};

template<>
class FclMarkerConverter<fcl::Cylinder>
{
    typedef boost::scoped_ptr<fcl::Cylinder> sPtrCylinder;

    private:
        //sPtrCylinder geoShape_;
        fcl::Cylinder geoShape_;

    public:
        FclMarkerConverter() : geoShape_(fcl::Cylinder(1.0, 1.0)) {}
        FclMarkerConverter(fcl::Cylinder &cyl) : geoShape_(cyl) {}

        void assignValues(visualization_msgs::Marker &marker)
        {
            marker.scale.x = this->geoShape_.radius;
            marker.scale.y = this->geoShape_.radius;
            marker.scale.z = this->geoShape_.lz;
            marker.type = visualization_msgs::Marker::CYLINDER;
        }

        fcl::Cylinder getGeoShape() const
        {
            return geoShape_;
        }
};


#endif /* FCL_MARKER_CONVERTER_HPP_ */
