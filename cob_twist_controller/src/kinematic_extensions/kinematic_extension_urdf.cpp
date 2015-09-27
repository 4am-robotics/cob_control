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
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This header contains the interface description for extening the
 *   kinematic chain with additional degrees of freedom, e.g. base_active or lookat
 *
 ****************************************************************/

#include <eigen_conversions/eigen_kdl.h>
#include "cob_twist_controller/kinematic_extensions/kinematic_extension_urdf.h"


/* BEGIN KinematicExtensionURDF ********************************************************************************************/
bool KinematicExtensionURDF::initUrdfExtension(std::string chain_base, std::string chain_tip)
{
    ///parse robot_description and generate KDL chains
    KDL::Tree tree;
    KDL::Chain chain;
    if (!kdl_parser::treeFromParam("robot_description", tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    tree.getChain(chain_base, chain_tip, chain);
    if(chain.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    for(unsigned int i=0; i<chain.getNrOfJoints(); i++)
    {
        joint_names_.push_back(chain.getSegment(i).getJoint().getName());
    }

    p_jnt2jac_ = new KDL::ChainJntToJacSolver(chain);
    return true;
}

void KinematicExtensionURDF::processResultExtension(const KDL::JntArray& q_dot_ik)
{
    std_msgs::Float64MultiArray command_msg;

    for(unsigned int i=0; i<joint_names_.size(); i++)
    {
        command_msg.data.push_back(q_dot_ik(params_.dof+i));
    }

    command_pub_.publish(command_msg);
}

void KinematicExtensionURDF::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = this->joint_states_.current_q_;
    KDL::JntArray q_dot_temp = this->joint_states_.current_q_dot_;

    //ToDo: Do we need more robust parsing/handling?
    for(uint16_t i = 0; i < joint_names_.size(); i++)
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


/* BEGIN KinematicExtensionTorso ****************************************************************************************/
KDL::Jacobian KinematicExtensionTorso::adjustJacobian(const KDL::Jacobian& jac_chain)
{
    ///compose jac_full considering kinematical extension
    KDL::Jacobian jac_full;
    KDL::Jacobian jac_extension;
    
    //calculate Jacobian for extension
    p_jnt2jac_->JntToJac(joint_states_.current_q_, jac_extension);
    
    //transform accordingly
    //ToDo
    
    //compose full Jacobian
    Matrix6Xd_t jac_full_matrix;
    jac_full_matrix.resize(6, jac_chain.data.cols() + jac_extension.data.cols());
    jac_full_matrix << jac_chain.data, jac_extension.data;
    jac_full.resize(jac_chain.data.cols() + jac_extension.data.cols());
    jac_full.data << jac_full_matrix;
    
    return jac_full;
}
/* END KinematicExtensionTorso ****************************************************************************************/


