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
#include "cob_twist_controller/constraint_solvers/solvers/unconstraint_solver.h"

/**
 * Implementation of a default solve-method for the inverse kinematics problem.
 * It calculates the pseudo-inverse of the Jacobian via the base implementation of calculatePinvJacobianBySVD.
 * With the pseudo-inverse the joint velocity vector is calculated.
 */
Eigen::MatrixXd UnconstraintSolver::solve(const Eigen::VectorXd& inCartVelocities, const KDL::JntArray& q, const KDL::JntArray& last_q_dot) const
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->jacobianData_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd jacobianPseudoInverse = this->calculatePinvJacobianBySVD(svd);
    Eigen::MatrixXd qdots_out = jacobianPseudoInverse * inCartVelocities;
    return qdots_out;
}

