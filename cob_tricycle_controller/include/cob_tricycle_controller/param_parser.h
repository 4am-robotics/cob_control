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


#ifndef COB_TRICYCLE_CONTROLLER_PARAM_PARSER_H
#define COB_TRICYCLE_CONTROLLER_PARAM_PARSER_H

#include <urdf/model.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

inline bool parseWheelTransform(const std::string& joint_name, const std::string& parent_link_name,
                         tf2::Transform &transform, urdf::Model* model)
{
    tf2::Transform transform_inc; transform_inc.setIdentity();
    if(model)
    {
        urdf::JointConstSharedPtr joint(model->getJoint(joint_name));
        if (!joint)
        {
            ROS_ERROR_STREAM(joint_name
                             << " couldn't be retrieved from model description");
            return false;
        }

        transform_inc = tf2::Transform(tf2::Quaternion(joint->parent_to_joint_origin_transform.rotation.x,
                                                       joint->parent_to_joint_origin_transform.rotation.y,
                                                       joint->parent_to_joint_origin_transform.rotation.z,
                                                       joint->parent_to_joint_origin_transform.rotation.w), 
                                       tf2::Vector3(joint->parent_to_joint_origin_transform.position.x,
                                                    joint->parent_to_joint_origin_transform.position.y,
                                                    joint->parent_to_joint_origin_transform.position.z));
        ROS_DEBUG_STREAM("transform_inc first");
        ROS_DEBUG_STREAM("Tx: "<<transform_inc.getOrigin().x()<<", Ty: "<<transform_inc.getOrigin().y()<<", Tz: "<<transform_inc.getOrigin().z());
        ROS_DEBUG_STREAM("Ax: "<<transform_inc.getRotation().getAxis().x()<<", Ay: "<<transform_inc.getRotation().getAxis().y()<<", Az: "<<transform_inc.getRotation().getAxis().z());
        ROS_DEBUG_STREAM("a: "<<transform_inc.getRotation().getAngle());
        while(joint->parent_link_name != parent_link_name)
        {
            urdf::LinkConstSharedPtr link_parent(model->getLink(joint->parent_link_name));
            ROS_DEBUG_STREAM("joint: "<<joint->name<<", parent_link_name: "<<joint->parent_link_name);
            if (!link_parent || !link_parent->parent_joint)
            {
                ROS_ERROR_STREAM(joint->parent_link_name
                                 << " couldn't be retrieved from model description or his parent joint");
                return false;
            }
            joint = link_parent->parent_joint;
            transform = tf2::Transform(tf2::Quaternion(joint->parent_to_joint_origin_transform.rotation.x,
                                                           joint->parent_to_joint_origin_transform.rotation.y,
                                                           joint->parent_to_joint_origin_transform.rotation.z,
                                                           joint->parent_to_joint_origin_transform.rotation.w), 
                                           tf2::Vector3(joint->parent_to_joint_origin_transform.position.x,
                                                        joint->parent_to_joint_origin_transform.position.y,
                                                        joint->parent_to_joint_origin_transform.position.z));
            transform_inc *= transform;
            ROS_DEBUG_STREAM("transform");
            ROS_DEBUG_STREAM("Tx: "<<transform.getOrigin().x()<<", Ty: "<<transform.getOrigin().y()<<", Tz: "<<transform.getOrigin().z());
            ROS_DEBUG_STREAM("Ax: "<<transform.getRotation().getAxis().x()<<", Ay: "<<transform.getRotation().getAxis().y()<<", Az: "<<transform.getRotation().getAxis().z());
            ROS_DEBUG_STREAM("a: "<<transform.getRotation().getAngle());
            ROS_DEBUG_STREAM("transform_inc");
            ROS_DEBUG_STREAM("Tx: "<<transform_inc.getOrigin().x()<<", Ty: "<<transform_inc.getOrigin().y()<<", Tz: "<<transform_inc.getOrigin().z());
            ROS_DEBUG_STREAM("Ax: "<<transform_inc.getRotation().getAxis().x()<<", Ay: "<<transform_inc.getRotation().getAxis().y()<<", Az: "<<transform_inc.getRotation().getAxis().z());
            ROS_DEBUG_STREAM("a: "<<transform_inc.getRotation().getAngle());
        }

        transform.setIdentity();
        transform = transform_inc;
        ROS_DEBUG_STREAM("final");
        ROS_DEBUG_STREAM("Tx: "<<transform.getOrigin().x()<<", Ty: "<<transform.getOrigin().y()<<", Tz: "<<transform.getOrigin().z());
        ROS_DEBUG_STREAM("Ax: "<<transform.getRotation().getAxis().x()<<", Ay: "<<transform.getRotation().getAxis().y()<<", Az: "<<transform.getRotation().getAxis().z());
        ROS_DEBUG_STREAM("a: "<<transform.getRotation().getAngle());
        return true;
    }
    else
        return false;
}

#endif  // COB_TRICYCLE_CONTROLLER_PARAM_PARSER_H
