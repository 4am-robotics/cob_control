/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
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
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This package provides the implementation of an inverse kinematics solver.
 *
 ****************************************************************/
#include "cob_twist_controller/inverse_differential_kinematics_solver.h"

#include <ros/ros.h>
#include <eigen_conversions/eigen_kdl.h>

/**
 * Solve the inverse kinematics problem at the first order differential level.
 */
int InverseDifferentialKinematicsSolver::CartToJnt(const JointStates& joint_states,
                                                   const KDL::Twist& v_in,
                                                   KDL::JntArray& qdot_out)
{
    int8_t retStat = -1;
    
    ///Let the ChainJntToJacSolver calculate the jacobian "jac_chain" for the current joint positions "q_in"
    KDL::Jacobian jac_chain(chain_.getNrOfJoints());
    jnt2jac_.JntToJac(joint_states.current_q_, jac_chain);
    
    ///append columns to Jacobian in order to reflect additional DoFs of kinematical extension
    this->jac_ = this->kinematic_extension_->adjustJacobian(jac_chain);

    Vector6d_t v_in_vec;
    tf::twistKDLToEigen(v_in, v_in_vec);

    Eigen::MatrixXd qdot_out_vec;
    retStat = constraint_solver_factory_.calculateJointVelocities(this->jac_.data,
                                                                  v_in_vec,
                                                                  joint_states,
                                                                  qdot_out_vec);
    
    ///convert output
    KDL::JntArray qdot_out_full(jac_.columns());
    for(int i = 0; i < jac_.columns(); i++)
    {
        qdot_out_full(i) = qdot_out_vec(i);
    }
    
    
    ///limiters shut be applied here in order to be able to consider the additional DoFs within "AllLimit", too
    qdot_out_full = this->limiters_->enforceLimits(qdot_out_full, joint_states.current_q_);
    
    ///process result for kinematical extension
    this->kinematic_extension_->processResultExtension(qdot_out_full);
    
    
    ///then qdot_out shut be resized to contain only the chain_qdot_out's again
    for(int i = 0; i < jac_chain.columns(); i++)
    {
        qdot_out(i) = qdot_out_full(i);
    }

    return retStat;
}

void InverseDifferentialKinematicsSolver::resetAll(TwistControllerParams params)
{   
    this->task_stack_controller_.clearAllTasks();
    this->params_ = params;
    if(0 != this->constraint_solver_factory_.resetAll(this->params_)) // params member as reference!!! else process will die!
    {
        ROS_ERROR("Failed to reset IDK constraint solver after dynamic_reconfigure.");
    }
    
    this->limiters_.reset(new LimiterContainer(this->params_, this->chain_));
    this->limiters_->init();
    
    this->kinematic_extension_.reset(KinematicExtensionBuilder::createKinematicExtension(this->params_));
}
