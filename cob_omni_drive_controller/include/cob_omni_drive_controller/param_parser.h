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


#ifndef COB_OMNI_DRIVE_CONTROLLER_PARAM_PARSER_H
#define COB_OMNI_DRIVE_CONTROLLER_PARAM_PARSER_H

#include <urdf/model.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

inline bool parseWheelTransform(const std::string& joint_name, const std::string& parent_link_name,
                         tf2::Transform &transform, urdf::Model* model)
{
    urdf::Pose transform_pose;
    if(model)
    {
        urdf::JointConstSharedPtr joint(model->getJoint(joint_name));
        if (!joint)
        {
            ROS_ERROR_STREAM(joint_name
                             << " couldn't be retrieved from model description");
            return false;
        }

        transform_pose.position = joint->parent_to_joint_origin_transform.position;
        transform_pose.rotation = joint->parent_to_joint_origin_transform.rotation;
        while(joint->parent_link_name != parent_link_name)
        {
            urdf::LinkConstSharedPtr link_parent(model->getLink(joint->parent_link_name));
            if (!link_parent || !link_parent->parent_joint)
            {
                ROS_ERROR_STREAM(joint->parent_link_name
                                 << " couldn't be retrieved from model description or his parent joint");
                return false;
            }
            joint = link_parent->parent_joint;
            transform_pose.position = transform_pose.position + joint->parent_to_joint_origin_transform.position;
            transform_pose.rotation = transform_pose.rotation * joint->parent_to_joint_origin_transform.rotation;
        }

        tf2::Transform rot(tf2::Quaternion(transform_pose.rotation.x,transform_pose.rotation.y,transform_pose.rotation.z,transform_pose.rotation.w),
                           tf2::Vector3(0,0,0));

        tf2::Transform trans; trans.setIdentity();
        trans.setOrigin(tf2::Vector3(transform_pose.position.x,transform_pose.position.y,transform_pose.position.z));

        transform.setIdentity();
        transform = rot * trans;
        return true;
    }
    else
        return false;
}

#endif  // COB_OMNI_DRIVE_CONTROLLER_PARAM_PARSER_H
