/*!
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
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Felix Messmer, email: felix.messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: November, 2015
 *
 * \brief
 *   This header contains the interface description for extening the
 *   kinematic chain with additional degrees of freedom, e.g. base_active or lookat
 *
 ****************************************************************/

#include <string>
#include <vector>
#include <limits>
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

    /// ToDo: orientation of lin_AXIS should be determined by a parameter
    KDL::Vector lookat_lin_axis(1.0, 0.0, 0.0);
    KDL::Joint lookat_lin_joint("lookat_lin_joint", KDL::Vector(), lookat_lin_axis, KDL::Joint::TransAxis);
    KDL::Segment lookat_rotx_link("lookat_rotx_link", lookat_lin_joint);
    chain_ext_.addSegment(lookat_rotx_link);
    limits_ext_max_.push_back(std::numeric_limits<double>::max());
    limits_ext_min_.push_back(-params_.limiter_params.limits_tolerance);
    limits_ext_vel_.push_back(std::numeric_limits<double>::max());
    limits_ext_acc_.push_back(std::numeric_limits<double>::max());

    KDL::Vector lookat_rotx_axis(1.0, 0.0, 0.0);
    KDL::Joint lookat_rotx_joint("lookat_rotx_joint", KDL::Vector(), lookat_rotx_axis, KDL::Joint::RotAxis);
    KDL::Segment lookat_roty_link("lookat_roty_link", lookat_rotx_joint);
    chain_ext_.addSegment(lookat_roty_link);
    limits_ext_max_.push_back(M_PI);
    limits_ext_min_.push_back(-M_PI);
    limits_ext_vel_.push_back(std::numeric_limits<double>::max());
    limits_ext_acc_.push_back(std::numeric_limits<double>::max());

    KDL::Vector lookat_roty_axis(0.0, 1.0, 0.0);
    KDL::Joint lookat_roty_joint("lookat_roty_joint", KDL::Vector(), lookat_roty_axis, KDL::Joint::RotAxis);
    KDL::Segment lookat_rotz_link("lookat_rotz_link", lookat_roty_joint);
    chain_ext_.addSegment(lookat_rotz_link);
    limits_ext_max_.push_back(M_PI);
    limits_ext_min_.push_back(-M_PI);
    limits_ext_vel_.push_back(std::numeric_limits<double>::max());
    limits_ext_acc_.push_back(std::numeric_limits<double>::max());

    KDL::Vector lookat_rotz_axis(0.0, 0.0, 1.0);
    KDL::Joint lookat_rotz_joint("lookat_rotz_joint", KDL::Vector(), lookat_rotz_axis, KDL::Joint::RotAxis);
    KDL::Segment lookat_focus_frame("lookat_focus_frame", lookat_rotz_joint);
    chain_ext_.addSegment(lookat_focus_frame);
    limits_ext_max_.push_back(M_PI);
    limits_ext_min_.push_back(-M_PI);
    limits_ext_vel_.push_back(std::numeric_limits<double>::max());
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

    integrator_.reset(new SimpsonIntegrator(ext_dof_));

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
            joint_states_ext_.current_q_(i) = pos[i];
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
