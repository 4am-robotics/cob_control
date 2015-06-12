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
 *   This header contains the description of the unconstraint solver
 *   Implements methods from constraint_solver_base
 *
 ****************************************************************/
#include "cob_twist_controller/constraint_solvers/solvers/gradient_projection_method_solver.h"

/**
 * Solve the inverse differential kinematics equation by using GPM.
 * In addtion to the partial solution q_dot = J^+ * v the homogeneous solution (I - J^+ * J) q_dot_0 is calculated.
 * The q_dot_0 results from the sum of the constraint cost function gradients. The terms of the sum are weighted with a factor k_H separately.
 */
Eigen::MatrixXd GradientProjectionMethodSolver::solve(const Vector6d &inCartVelocities,
                                      const KDL::JntArray& q,
                                      const KDL::JntArray& last_q_dot) const
{
    double k_H;
    Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(q.rows());
    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
    Eigen::MatrixXd partialSolution = jacobianPseudoInverse * inCartVelocities;
    Eigen::MatrixXd homogeneousSolution = Eigen::MatrixXd::Zero(partialSolution.rows(), partialSolution.cols());

    for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        q_dot_0 = (*it)->getPartialValues();
        Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
        k_H = (*it)->getSelfMotionMagnitude(partialSolution, tmpHomogeneousSolution);
        homogeneousSolution += (k_H * tmpHomogeneousSolution);
    }

    if(q_dot_0.norm() > 0.00001)
    //if(false)
    {
        ROS_INFO_STREAM("jacobianPseudoInverse: " << std::endl << jacobianPseudoInverse);
        ROS_INFO_STREAM("this->jacobianData_: " << std::endl << this->jacobian_data_);
        ROS_INFO_STREAM("projector: " << std::endl << projector);
        ROS_INFO_STREAM("q_dot_0: " << std::endl << q_dot_0.transpose());
        ROS_INFO_STREAM("Last k_H: " << std::endl << k_H);
        ROS_INFO_STREAM("partialSolution: " << std::endl << partialSolution);
        ROS_INFO_STREAM("homogeneousSolution: " << std::endl << homogeneousSolution);
    }

    Eigen::MatrixXd qdots_out = partialSolution + homogeneousSolution; // weighting with k_H is done in loop
    return qdots_out;
}
