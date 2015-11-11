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

    tree.getChain(params_.chain_base_link, params_.chain_tip_link, chain_);
    if (chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    //ToDo:
    ///orientation of lin_AXIS should be determined by a parameter
    ///append additional segments to chain_ for Lookat (i.e. lin_AXIS, rot_X, rot_Y, rot_Z)
    
    jnt2jac_.reset(new KDL::ChainJntToJacSolver(chain_));

    this->ext_dof_ = 4;
    this->joint_states_.last_q_.resize(ext_dof_);
    this->joint_states_.last_q_dot_.resize(ext_dof_);
    this->joint_states_.current_q_.resize(ext_dof_);
    this->joint_states_.current_q_dot_.resize(ext_dof_);

    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        limits_max_.push_back(std::numeric_limits<double>::max());
        limits_min_.push_back(0.0);
        limits_vel_.push_back(std::numeric_limits<double>::max());
        limits_acc_.push_back(std::numeric_limits<double>::max());
    }

    return true;
}

KDL::Jacobian KinematicExtensionLookat::adjustJacobian(const KDL::Jacobian& jac_chain)
{
    /// compose jac_full considering kinematical extension
    KDL::Jacobian jac_full;
    
    jnt2jac_->JntToJac(joint_states_full_.current_q_ ,jac_full);

    return jac_full;
}

JointStates KinematicExtensionLookat::adjustJointStates(const JointStates& joint_states)
{
    joint_states_.last_q_ = joint_states_.current_q_;
    joint_states_.last_q_dot_ = joint_states_.current_q_dot_;
    
    //ToDo: calculate new current integration 



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
        joint_states_full_.current_q_(chain_dof + i) = this->joint_states_.current_q_(i);
        joint_states_full_.last_q_(chain_dof + i) = this->joint_states_.last_q_(i);
        joint_states_full_.current_q_dot_(chain_dof + i) = this->joint_states_.current_q_dot_(i);
        joint_states_full_.last_q_dot_(chain_dof + i) = this->joint_states_.last_q_dot_(i);
    }

    return joint_states_full_;
}

LimiterParams KinematicExtensionLookat::adjustLimiterParams(const LimiterParams& limiter_params)
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

void KinematicExtensionLookat::processResultExtension(const KDL::JntArray& q_dot_ik)
{
    //ToDo: set new velocity!
}

/* END KinematicExtensionLookat ********************************************************************************************/
