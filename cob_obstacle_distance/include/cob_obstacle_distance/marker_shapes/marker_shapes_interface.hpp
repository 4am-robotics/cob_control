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
 *   This header contains the interface definition to manage
 *   MarkerShapes in common containers / vectors.
 *
 ****************************************************************/

#ifndef MARKER_SHAPES_INTERFACE_HPP_
#define MARKER_SHAPES_INTERFACE_HPP_

#include <boost/shared_ptr.hpp>
#include <stdint.h>
#include <visualization_msgs/Marker.h>
#include <fcl/collision_object.h>
#include <fcl/BVH/BVH_model.h>

/* BEGIN IMarkerShape *******************************************************************************************/
/// Interface class marking methods that have to be implemented in derived classes.
class IMarkerShape
{
    protected:
        static uint32_t class_ctr_;

    public:
         IMarkerShape();
         virtual uint32_t getId() const = 0;
         virtual void setColor(double color_r, double color_g, double color_b, double color_a = 1.0) = 0;
         virtual visualization_msgs::Marker getMarker() = 0;
         virtual void setDrawn() = 0;
         virtual bool isDrawn() const = 0;
         virtual void updatePose(const geometry_msgs::Vector3& pos, const geometry_msgs::Quaternion& quat) = 0;
         virtual void updatePose(const geometry_msgs::Pose& pose) = 0;
         virtual fcl::CollisionObject getCollisionObject() const = 0;
         virtual geometry_msgs::Pose getMarkerPose() const = 0;
         virtual geometry_msgs::Pose getOriginRelToFrame() const = 0;
         virtual ~IMarkerShape() {}
};
/* END IMarkerShape *********************************************************************************************/

typedef boost::shared_ptr< IMarkerShape > PtrIMarkerShape_t;
typedef fcl::BVHModel<fcl::RSS> BVH_RSS_t;

#endif /* MARKER_SHAPES_INTERFACE_HPP_ */
