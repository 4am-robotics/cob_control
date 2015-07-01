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
int InverseDifferentialKinematicsSolver::CartToJnt(const KDL::JntArray& q_in,
                                                   const KDL::JntArray& last_q_dot,
                                                   const KDL::Twist& v_in,
                                                   KDL::JntArray& qdot_out)
{
    int8_t retStat = -1;
    
    ///Let the ChainJntToJacSolver calculate the jacobian "jac_chain" for the current joint positions "q_in"
    KDL::Jacobian jac_chain(chain_.getNrOfJoints());
    jnt2jac_.JntToJac(q_in, jac_chain);
    
    ///append columns to Jacobian in order to reflect additional DoFs of kinematical extension
    this->jac_ = this->kinematic_extension_->adjust_jacobian(jac_chain);
    ROS_DEBUG_STREAM("Jacobian:" << std::endl << this->jac_.data);

    t_Vector6d v_in_vec;
    tf::twistKDLToEigen(v_in, v_in_vec);

    Eigen::MatrixXd qdot_out_vec;
    retStat = constraint_solver_factory_.calculateJointVelocities(this->params_,
                                                                  this->jac_.data,
                                                                  v_in_vec,
                                                                  q_in,
                                                                  last_q_dot,
                                                                  qdot_out_vec);
    
    ///convert output
    KDL::JntArray qdot_out_full(jac_.columns());
    for(int i = 0; i < jac_.columns(); i++)
    {
        qdot_out_full(i) = qdot_out_vec(i);
    }
    
    //ToDo:
    ///limiters shut be applied here in order to be able to consider the additional DoFs within "AllLimit", too
    
    
    ///process result for kinematical extension
    this->kinematic_extension_->process_result_extension(qdot_out_full);
    
    
    ///then qdot_out shut be resized to contain only the chain_qdot_out's again
    for(int i = 0; i < jac_chain.columns(); i++)
    {
        qdot_out(i) = qdot_out_full(i);
    }

    return retStat;
}
