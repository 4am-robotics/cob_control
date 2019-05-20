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
#include <vector>
#include <limits>
#include <algorithm>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_kdl.h>
#include "cob_twist_controller/kinematic_extensions/kinematic_extension_lookat.h"

/* BEGIN KinematicExtensionLookat ********************************************************************************************/
bool KinematicExtensionLookat::initExtension()
{
    /// parse robot_description and generate KDL chains
    KDL::Tree tree;
    if (!kdl_parser::treeFromParam("robot_description", tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    KDL::Chain chain_main;
    tree.getChain(params_.chain_base_link, params_.chain_tip_link, chain_main);
    if (chain_main.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    KDL::Joint::JointType lookat_lin_joint_type = KDL::Joint::None;
    switch (params_.lookat_offset.lookat_axis_type)
    {
        case X_POSITIVE:
            lookat_lin_joint_type = KDL::Joint::TransX;
            break;
        case Y_POSITIVE:
            lookat_lin_joint_type = KDL::Joint::TransY;
            break;
        case Z_POSITIVE:
            lookat_lin_joint_type = KDL::Joint::TransZ;
            break;
        case X_NEGATIVE:
            lookat_lin_joint_type = KDL::Joint::TransX;
            ROS_ERROR("X_NEGATIVE axis_type not supported");
            return false;
        case Y_NEGATIVE:
            lookat_lin_joint_type = KDL::Joint::TransY;
            ROS_ERROR("Y_NEGATIVE axis_type not supported");
            return false;
        case Z_NEGATIVE:
            lookat_lin_joint_type = KDL::Joint::TransZ;
            ROS_ERROR("Z_NEGATIVE axis_type not supported");
            return false;
        default:
            ROS_ERROR("LookatAxisType %d not defined! Using default: 'X_POSITIVE'!", params_.lookat_offset.lookat_axis_type);
            lookat_lin_joint_type = KDL::Joint::TransX;
            break;
    }

    KDL::Frame offset;
    try
    {
        tf::StampedTransform offset_transform;
        tf_listener_.lookupTransform(params_.chain_tip_link, params_.lookat_pointing_frame, ros::Time(0), offset_transform);
        tf::transformTFToKDL(offset_transform, offset);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("LookatAction: %s", ex.what());
        ROS_WARN_STREAM("Using 'lookat_offset' instead");
        offset.p = KDL::Vector(params_.lookat_offset.translation_x, params_.lookat_offset.translation_y, params_.lookat_offset.translation_z);
        offset.M = KDL::Rotation::Quaternion(params_.lookat_offset.rotation_x, params_.lookat_offset.rotation_y, params_.lookat_offset.rotation_z, params_.lookat_offset.rotation_w);
    }

    //fixed pointing offset
    KDL::Joint offset_joint("offset_joint", KDL::Joint::None);
    KDL::Segment offset_link("offset_link", offset_joint, offset);
    chain_ext_.addSegment(offset_link);

    //lookat chain
    KDL::Joint lookat_lin_joint("lookat_lin_joint", lookat_lin_joint_type);
    KDL::Segment lookat_rotx_link("lookat_rotx_link", lookat_lin_joint);
    chain_ext_.addSegment(lookat_rotx_link);
    limits_ext_max_.push_back(std::numeric_limits<double>::max());
    limits_ext_min_.push_back(-std::numeric_limits<double>::max());
    limits_ext_vel_.push_back(5.0);
    limits_ext_acc_.push_back(std::numeric_limits<double>::max());

    KDL::Joint lookat_rotx_joint("lookat_rotx_joint", KDL::Joint::RotX);
    KDL::Segment lookat_roty_link("lookat_roty_link", lookat_rotx_joint);
    chain_ext_.addSegment(lookat_roty_link);
    // limits_ext_max_.push_back(M_PI);
    // limits_ext_min_.push_back(-M_PI);
    limits_ext_max_.push_back(std::numeric_limits<double>::max());
    limits_ext_min_.push_back(-std::numeric_limits<double>::max());
    // limits_ext_vel_.push_back(std::numeric_limits<double>::max());
    limits_ext_vel_.push_back(M_PI);
    limits_ext_acc_.push_back(std::numeric_limits<double>::max());

    KDL::Joint lookat_roty_joint("lookat_roty_joint", KDL::Joint::RotY);
    KDL::Segment lookat_rotz_link("lookat_rotz_link", lookat_roty_joint);
    chain_ext_.addSegment(lookat_rotz_link);
    // limits_ext_max_.push_back(M_PI);
    // limits_ext_min_.push_back(-M_PI);
    limits_ext_max_.push_back(std::numeric_limits<double>::max());
    limits_ext_min_.push_back(-std::numeric_limits<double>::max());
    // limits_ext_vel_.push_back(std::numeric_limits<double>::max());
    limits_ext_vel_.push_back(M_PI);
    limits_ext_acc_.push_back(std::numeric_limits<double>::max());

    KDL::Joint lookat_rotz_joint("lookat_rotz_joint", KDL::Joint::RotZ);
    KDL::Segment lookat_focus_frame("lookat_focus_frame", lookat_rotz_joint);
    chain_ext_.addSegment(lookat_focus_frame);
    // limits_ext_max_.push_back(M_PI);
    // limits_ext_min_.push_back(-M_PI);
    limits_ext_max_.push_back(std::numeric_limits<double>::max());
    limits_ext_min_.push_back(-std::numeric_limits<double>::max());
    // limits_ext_vel_.push_back(std::numeric_limits<double>::max());
    limits_ext_vel_.push_back(M_PI);
    limits_ext_acc_.push_back(std::numeric_limits<double>::max());

    chain_full_ = chain_main;
    chain_full_.addChain(chain_ext_);

    fk_solver_ext_.reset(new KDL::ChainFkSolverPos_recursive(chain_ext_));
    jnt2jac_.reset(new KDL::ChainJntToJacSolver(chain_full_));

    this->ext_dof_ = 4;
    this->joint_states_ext_.last_q_.resize(ext_dof_);
    KDL::SetToZero(this->joint_states_ext_.last_q_);
    this->joint_states_ext_.last_q_dot_.resize(ext_dof_);
    KDL::SetToZero(this->joint_states_ext_.last_q_dot_);
    this->joint_states_ext_.current_q_.resize(ext_dof_);
    KDL::SetToZero(this->joint_states_ext_.current_q_);
    this->joint_states_ext_.current_q_dot_.resize(ext_dof_);
    KDL::SetToZero(this->joint_states_ext_.current_q_dot_);

    integrator_.reset(new SimpsonIntegrator(ext_dof_, 0.2));  // default smoothing: 0.2

    timer_ = nh_.createTimer(ros::Duration(1/100.0), &KinematicExtensionLookat::broadcastFocusFrame, this);
    timer_.start();

    return true;
}

KDL::Jacobian KinematicExtensionLookat::adjustJacobian(const KDL::Jacobian& jac_chain)
{
    /// compose jac_full considering kinematical extension
    boost::mutex::scoped_lock lock(mutex_);
    KDL::Jacobian jac_full(chain_full_.getNrOfJoints());

    jnt2jac_->JntToJac(joint_states_full_.current_q_ , jac_full);

    return jac_full;
}

JointStates KinematicExtensionLookat::adjustJointStates(const JointStates& joint_states)
{
    boost::mutex::scoped_lock lock(mutex_);
    unsigned int chain_dof = joint_states.current_q_.rows();
    joint_states_full_.current_q_.resize(chain_dof + ext_dof_);
    joint_states_full_.last_q_.resize(chain_dof + ext_dof_);
    joint_states_full_.current_q_dot_.resize(chain_dof + ext_dof_);
    joint_states_full_.last_q_dot_.resize(chain_dof + ext_dof_);

    for (unsigned int i = 0; i< chain_dof; i++)
    {
        joint_states_full_.current_q_(i) = joint_states.current_q_(i);
        joint_states_full_.last_q_(i) = joint_states.last_q_(i);
        joint_states_full_.current_q_dot_(i) = joint_states.current_q_dot_(i);
        joint_states_full_.last_q_dot_(i) = joint_states.last_q_dot_(i);
    }
    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        joint_states_full_.current_q_(chain_dof + i) = this->joint_states_ext_.current_q_(i);
        joint_states_full_.last_q_(chain_dof + i) = this->joint_states_ext_.last_q_(i);
        joint_states_full_.current_q_dot_(chain_dof + i) = this->joint_states_ext_.current_q_dot_(i);
        joint_states_full_.last_q_dot_(chain_dof + i) = this->joint_states_ext_.last_q_dot_(i);
    }

    return joint_states_full_;
}

LimiterParams KinematicExtensionLookat::adjustLimiterParams(const LimiterParams& limiter_params)
{
    LimiterParams lp = limiter_params;
    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        lp.limits_max.push_back(limits_ext_max_[i]);
        lp.limits_min.push_back(limits_ext_min_[i]);
        lp.limits_vel.push_back(limits_ext_vel_[i]);
        lp.limits_acc.push_back(limits_ext_acc_[i]);
    }
    return lp;
}

void KinematicExtensionLookat::processResultExtension(const KDL::JntArray& q_dot_ik)
{
    /// update JointStates using integration
    boost::mutex::scoped_lock lock(mutex_);
    std::vector<double> pos;
    std::vector<double> vel;

    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        joint_states_ext_.current_q_dot_(i) = q_dot_ik(params_.dof + i);
    }

    if (integrator_->updateIntegration(joint_states_ext_.current_q_dot_, joint_states_ext_.current_q_, pos, vel))
    {
        for (unsigned int i = 0; i < ext_dof_; i++)
        {
            joint_states_ext_.last_q_(i) = this->joint_states_ext_.current_q_(i);
            joint_states_ext_.last_q_dot_(i) = this->joint_states_ext_.current_q_dot_(i);
            joint_states_ext_.current_q_(i) = std::max(pos[i], 0.1);  // do not look backwards
            joint_states_ext_.current_q_dot_(i) = vel[i];
        }
    }
}

void KinematicExtensionLookat::broadcastFocusFrame(const ros::TimerEvent& event)
{
    boost::mutex::scoped_lock lock(mutex_);
    KDL::Frame focus_frame;

    fk_solver_ext_->JntToCart(joint_states_ext_.current_q_, focus_frame);

    tf::Transform transform;
    tf::transformKDLToTF(focus_frame, transform);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), params_.chain_tip_link, "lookat_focus_frame"));
}

/* END KinematicExtensionLookat ********************************************************************************************/
