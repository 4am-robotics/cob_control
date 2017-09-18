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
        visualization_msgs::Marker marker_;
        geometry_msgs::Pose origin_;
        bool drawable_; ///> If the marker shape is even drawable or not.

    public:
         IMarkerShape();
         virtual uint32_t getId() const = 0;
         virtual void setColor(double color_r, double color_g, double color_b, double color_a = 1.0) = 0;
         virtual visualization_msgs::Marker getMarker() = 0;
         virtual void updatePose(const geometry_msgs::Vector3& pos, const geometry_msgs::Quaternion& quat) = 0;
         virtual void updatePose(const geometry_msgs::Pose& pose) = 0;
         virtual fcl::CollisionObject getCollisionObject() const = 0;
         virtual geometry_msgs::Pose getMarkerPose() const = 0;
         virtual geometry_msgs::Pose getOriginRelToFrame() const = 0;


         /**
          * Decide whether the marker shape can be drawn or not. E.g. self collision frames need not to be drawn again as they are
          * available in rviz -> robot model -> collision enabled.
          * @param can_be_drawn: Decide whether the marker shall be drawn or not.
          */
         inline void setDrawable(bool can_be_drawn)
         {
             this->drawable_ = can_be_drawn;
         }

         inline bool isDrawable()
         {
             return this->drawable_;
         }

         virtual ~IMarkerShape() {}
};
/* END IMarkerShape *********************************************************************************************/

typedef std::shared_ptr< IMarkerShape > PtrIMarkerShape_t;
typedef fcl::BVHModel<fcl::RSS> BVH_RSS_t;

#endif /* MARKER_SHAPES_INTERFACE_HPP_ */
