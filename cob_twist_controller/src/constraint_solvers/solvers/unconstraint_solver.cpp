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
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: March, 2015
 *
 * \brief
 *   Implementation of an unconstraint solver.
 *
 ****************************************************************/

#include <ros/ros.h>

#include "cob_twist_controller/constraint_solvers/solvers/unconstraint_solver.h"

/**
 * Implementation of a default solve-method for the inverse kinematics problem.
 * It calculates the pseudo-inverse of the Jacobian via the base implementation of calculatePinvJacobianBySVD.
 * With the pseudo-inverse the joint velocity vector is calculated.
 */
Eigen::MatrixXd UnconstraintSolver::solve(const Vector6d_t& in_cart_velocities,
                                          const JointStates& joint_states)
{
    Eigen::MatrixXd pinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd qdots_out = pinv * in_cart_velocities;
    return qdots_out;
}

