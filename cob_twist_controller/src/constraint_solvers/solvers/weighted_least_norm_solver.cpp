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
 *   Implementation of an JLA solver.
 *   Special constraint: Avoid joint limits.
 *
 ****************************************************************/
#include "ros/ros.h"
#include "cob_twist_controller/constraint_solvers/solvers/weighted_least_norm_solver.h"


/**
 * Specific implementation of the solve method using a weighted least norm.
 * This is done by calculation of a weighting which is dependent on inherited classes for the Jacobian.
 * Uses the base implementation of calculatePinvJacobianBySVD to calculate the pseudo-inverse (weighted) Jacobian.
 */
Eigen::MatrixXd WeightedLeastNormSolver::solve(const Eigen::VectorXd &inCartVelocities, const KDL::JntArray& q, const KDL::JntArray& last_q_dot) const
{
    Eigen::MatrixXd W_WLN = this->calculateWeighting(q, last_q_dot);
    // for the following formulas see Chan paper ISSN 1042-296X [Page 288]
    Eigen::MatrixXd root_W_WLN =  W_WLN.cwiseSqrt(); // element-wise sqrt -> ok because diag matrix W^(1/2)
    Eigen::MatrixXd inv_root_W_WLN =  root_W_WLN.inverse(); // -> W^(-1/2)

    // SVD of JLA weighted Jacobian: Damping will be done later in calculatePinvJacobianBySVD for pseudo-inverse Jacobian with additional truncation etc.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->jacobianData_ * inv_root_W_WLN,
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd weightedJacobianPseudoInverse = this->calculatePinvJacobianBySVD(svd);
    // Take care: W^(1/2) * q_dot = weighted_pinv_J * x_dot -> One must consider the weighting!!!
    Eigen::MatrixXd qdots_out = inv_root_W_WLN * weightedJacobianPseudoInverse * inCartVelocities;
    return qdots_out;
}

/**
 * This function returns the identity as weighting matrix for base functionality.
 */
Eigen::MatrixXd WeightedLeastNormSolver::calculateWeighting(const KDL::JntArray& q, const KDL::JntArray& last_q_dot) const
{
    uint32_t cols = this->jacobianData_.cols();
    Eigen::VectorXd output = Eigen::VectorXd::Ones(cols);
    return output.asDiagonal();
}
