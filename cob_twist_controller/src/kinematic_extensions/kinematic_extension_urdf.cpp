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


#include <string>
#include <limits>
#include <eigen_conversions/eigen_kdl.h>
#include "cob_twist_controller/kinematic_extensions/kinematic_extension_urdf.h"

/* BEGIN KinematicExtensionURDF ********************************************************************************************/
bool KinematicExtensionURDF::initExtension()
{
    /// parse robot_description and generate KDL chains
    KDL::Tree tree;
    if (!kdl_parser::treeFromParam("robot_description", tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    tree.getChain(ext_base_, ext_tip_, chain_);
    if (chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++)
    {
        ROS_DEBUG_STREAM("Segment[" << i << "] Name : " << chain_.getSegment(i).getName());
        ROS_DEBUG_STREAM("Joint[" << i << "] Name: " << chain_.getSegment(i).getJoint().getName());
        ROS_DEBUG_STREAM("Joint[" << i << "] Type: " << chain_.getSegment(i).getJoint().getTypeName());
        if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
        {
            ROS_DEBUG_STREAM("Adding Joint " << chain_.getSegment(i).getJoint().getName());
            joint_names_.push_back(chain_.getSegment(i).getJoint().getName());
        }
    }
    this->ext_dof_ = chain_.getNrOfJoints();
    this->joint_states_.last_q_.resize(ext_dof_);
    this->joint_states_.last_q_dot_.resize(ext_dof_);
    this->joint_states_.current_q_.resize(ext_dof_);
    this->joint_states_.current_q_dot_.resize(ext_dof_);

    /// parse robot_description and set velocity limits
    urdf::Model model;
    if (!model.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file for JointLimits");
        return false;
    }

    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        limits_max_.push_back(model.getJoint(joint_names_[i])->limits->upper);
        limits_min_.push_back(model.getJoint(joint_names_[i])->limits->lower);
        limits_vel_.push_back(model.getJoint(joint_names_[i])->limits->velocity);
        limits_acc_.push_back(std::numeric_limits<double>::max());
    }

    return true;
}

KDL::Jacobian KinematicExtensionURDF::adjustJacobian(const KDL::Jacobian& jac_chain)
{
    /// compose jac_full considering kinematical extension
    KDL::Jacobian jac_full;

    // jacobian matrix for the extension
    Eigen::Matrix<double, 6, Eigen::Dynamic> jac_ext;
    jac_ext.resize(6, ext_dof_);
    jac_ext.setZero();

    unsigned int k = 0;
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++)
    {
        KDL::Joint joint = chain_.getSegment(i).getJoint();
        double eps_type = -1.0;  // joint_type selector: 0.0 -> rot_axis, 1.0 -> lin_axis

        switch (joint.getType())
        {
            // revolute axis
            case KDL::Joint::RotAxis:
            case KDL::Joint::RotX:
            case KDL::Joint::RotY:
            case KDL::Joint::RotZ:
                eps_type = 0.0;
                break;
            // linear axis
            case KDL::Joint::TransAxis:
            case KDL::Joint::TransX:
            case KDL::Joint::TransY:
            case KDL::Joint::TransZ:
                eps_type = 1.0;
                break;
            // fixed axis
            case KDL::Joint::None:
            default:
                break;
        }

        if (eps_type < 0.0)
        {
            ROS_DEBUG_STREAM("Not considering " << joint.getName() << " in jac_ext");
            continue;
        }

        /// get required transformations
        tf::StampedTransform eb_transform_ct, cb_transform_eb;
        KDL::Frame eb_frame_ct, cb_frame_eb;
        try
        {
            ros::Time now = ros::Time::now();
            tf_listener_.waitForTransform(chain_.getSegment(i).getName(), params_.chain_tip_link, now, ros::Duration(0.5));
            tf_listener_.lookupTransform(chain_.getSegment(i).getName(), params_.chain_tip_link,  now, eb_transform_ct);

            tf_listener_.waitForTransform(params_.chain_base_link, chain_.getSegment(i).getName(), now, ros::Duration(0.5));
            tf_listener_.lookupTransform(params_.chain_base_link, chain_.getSegment(i).getName(), now, cb_transform_eb);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        eb_frame_ct.p = KDL::Vector(eb_transform_ct.getOrigin().x(), eb_transform_ct.getOrigin().y(), eb_transform_ct.getOrigin().z());
        eb_frame_ct.M = KDL::Rotation::Quaternion(eb_transform_ct.getRotation().x(), eb_transform_ct.getRotation().y(), eb_transform_ct.getRotation().z(), eb_transform_ct.getRotation().w());

        cb_frame_eb.p = KDL::Vector(cb_transform_eb.getOrigin().x(), cb_transform_eb.getOrigin().y(), cb_transform_eb.getOrigin().z());
        cb_frame_eb.M = KDL::Rotation::Quaternion(cb_transform_eb.getRotation().x(), cb_transform_eb.getRotation().y(), cb_transform_eb.getRotation().z(), cb_transform_eb.getRotation().w());

        // rotation from base_frame of primary chain to base_frame of extension (eb)
        Eigen::Quaterniond quat_cb;
        tf::quaternionKDLToEigen(cb_frame_eb.M, quat_cb);
        Eigen::Matrix3d rot_cb = quat_cb.toRotationMatrix();

        /// angular velocities
        Eigen::Vector3d axis_eb(joint.JointAxis().x(), joint.JointAxis().y(), joint.JointAxis().z());  // axis wrt eb
        Eigen::Vector3d axis_cb = quat_cb * axis_eb;  // transform to cb

        /// linear velocities
        // vector from base_frame of extension (eb) to endeffector (ct)
        Eigen::Vector3d p_eb(eb_frame_ct.p.x(), eb_frame_ct.p.y(), eb_frame_ct.p.z());
        Eigen::Vector3d p_cb = quat_cb * p_eb;                  // transform to cb
        Eigen::Vector3d axis_cross_p_cb = axis_cb.cross(p_cb);  // tangential velocity

        /// explicit form of jacobian
        jac_ext(0, k) = eps_type * axis_cb(0) + (1.0 - eps_type) * axis_cross_p_cb(0);
        jac_ext(1, k) = eps_type * axis_cb(0) + (1.0 - eps_type) * axis_cross_p_cb(1);
        jac_ext(2, k) = eps_type * axis_cb(0) + (1.0 - eps_type) * axis_cross_p_cb(2);
        jac_ext(3, k) = eps_type * 0.0     + (1.0 - eps_type) * axis_cb(0);
        jac_ext(4, k) = eps_type * 0.0     + (1.0 - eps_type) * axis_cb(1);
        jac_ext(5, k) = eps_type * 0.0     + (1.0 - eps_type) * axis_cb(2);
        k++;
    }

    // scale with extension_ratio
    jac_ext *= params_.extension_ratio;

    // combine Jacobian of primary chain and extension
    Matrix6Xd_t jac_full_matrix;
    jac_full_matrix.resize(6, jac_chain.data.cols() + jac_ext.cols());
    jac_full_matrix << jac_chain.data, jac_ext;
    jac_full.resize(jac_chain.data.cols() + jac_ext.cols());
    jac_full.data << jac_full_matrix;

    return jac_full;
}

JointStates KinematicExtensionURDF::adjustJointStates(const JointStates& joint_states)
{
    JointStates js;
    unsigned int chain_dof = joint_states.current_q_.rows();
    js.current_q_.resize(chain_dof + ext_dof_);
    js.last_q_.resize(chain_dof + ext_dof_);
    js.current_q_dot_.resize(chain_dof + ext_dof_);
    js.last_q_dot_.resize(chain_dof + ext_dof_);

    for (unsigned int i = 0; i< chain_dof; i++)
    {
        js.current_q_(i) = joint_states.current_q_(i);
        js.last_q_(i) = joint_states.last_q_(i);
        js.current_q_dot_(i) = joint_states.current_q_dot_(i);
        js.last_q_dot_(i) = joint_states.last_q_dot_(i);
    }
    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        js.current_q_(chain_dof + i) = this->joint_states_.current_q_(i);
        js.last_q_(chain_dof + i) = this->joint_states_.last_q_(i);
        js.current_q_dot_(chain_dof + i) = this->joint_states_.current_q_dot_(i);
        js.last_q_dot_(chain_dof + i) = this->joint_states_.last_q_dot_(i);
    }

    return js;
}

LimiterParams KinematicExtensionURDF::adjustLimiterParams(const LimiterParams& limiter_params)
{
    LimiterParams lp = limiter_params;
    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        lp.limits_max.push_back(limits_max_[i]);
        lp.limits_min.push_back(limits_min_[i]);
        lp.limits_vel.push_back(limits_vel_[i]);
        lp.limits_acc.push_back(limits_acc_[i]);
    }
    return lp;
}

void KinematicExtensionURDF::processResultExtension(const KDL::JntArray& q_dot_ik)
{
    std_msgs::Float64MultiArray command_msg;

    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        command_msg.data.push_back(q_dot_ik(params_.dof+i));
    }

    command_pub_.publish(command_msg);
}

void KinematicExtensionURDF::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = this->joint_states_.current_q_;
    KDL::JntArray q_dot_temp = this->joint_states_.current_q_dot_;

    // ToDo: Do we need more robust parsing/handling?
    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        q_temp(i) = msg->position[i];
        q_dot_temp(i) = msg->velocity[i];
    }

    this->joint_states_.last_q_ = joint_states_.current_q_;
    this->joint_states_.last_q_dot_ = joint_states_.current_q_dot_;
    this->joint_states_.current_q_ = q_temp;
    this->joint_states_.current_q_dot_ = q_dot_temp;
}
/* END KinematicExtensionURDF ********************************************************************************************/
