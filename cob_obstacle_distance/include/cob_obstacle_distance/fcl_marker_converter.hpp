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


#ifndef FCL_MARKER_CONVERTER_HPP_
#define FCL_MARKER_CONVERTER_HPP_

#include <boost/scoped_ptr.hpp>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>


#include "cob_obstacle_distance/marker_shapes/marker_shapes_interface.hpp"

// the less the better performance for BVH generation and distance calculation
// E.g. values == 100u then CPU% at 20 Hz is 65 %
// E.g. values == 10u then CPU% at 20 Hz is 35 %
#define SEGMENTS 10u
#define RINGS 10u
#define SEG_AXIS 10u
#define SEG_CIRCLE 10u

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
        FclMarkerConverter(fcl::Box& box) : geo_shape_(box) {}

        void assignValues(visualization_msgs::Marker& marker)
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

        void getBvhModel(BVH_RSS_t& bvh) const
        {
            const fcl::Transform3f x;
            fcl::generateBVHModel(bvh, geo_shape_, x);
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
            marker.scale.x = this->geo_shape_.radius * 2.0; // marker needs the diameter in x
            marker.scale.y = this->geo_shape_.radius * 2.0; // marker needs the diameter in y
            marker.scale.z = this->geo_shape_.radius * 2.0; // marker needs the diameter in z
            marker.type = visualization_msgs::Marker::SPHERE;
        }

        fcl::Sphere getGeoShape() const
        {
            return geo_shape_;
        }

        /**
         * Creates a BVH model from the fcl sphere.
         * Transform must not be given here. This is done when the collision object of the BVH model is requested.
         * @param bvh A reference to an already existing BVH model.
         * @return
         */
        void getBvhModel(BVH_RSS_t& bvh) const
        {
            const fcl::Transform3f x;
            fcl::generateBVHModel(bvh, geo_shape_, x, SEGMENTS, RINGS);
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
            marker.scale.x = this->geo_shape_.radius * 2.0; // marker needs the diameter in x
            marker.scale.y = this->geo_shape_.radius * 2.0; // marker needs the diameter in y
            marker.scale.z = this->geo_shape_.lz;
            marker.type = visualization_msgs::Marker::CYLINDER;
        }

        fcl::Cylinder getGeoShape() const
        {
            return geo_shape_;
        }

        void getBvhModel(BVH_RSS_t& bvh) const
        {
            const fcl::Transform3f x;
            fcl::generateBVHModel(bvh, geo_shape_, x, SEG_CIRCLE, SEG_AXIS);
        }
};

#endif /* FCL_MARKER_CONVERTER_HPP_ */
