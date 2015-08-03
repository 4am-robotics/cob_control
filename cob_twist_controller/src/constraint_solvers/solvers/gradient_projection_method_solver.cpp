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

#include <cmath>

#include "cob_twist_controller/constraints/self_motion_magnitude.h"

/**
 * Solve the inverse differential kinematics equation by using GPM.
 * In addtion to the partial solution q_dot = J^+ * v the homogeneous solution (I - J^+ * J) q_dot_0 is calculated.
 * The q_dot_0 results from the sum of the constraint cost function gradients. The terms of the sum are weighted with a factor k_H separately.
 */
Eigen::MatrixXd GradientProjectionMethodSolver::solve(const Vector6d_t& inCartVelocities,
                                                      const JointStates& joint_states)
{
    double activation_gain;
    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
    Eigen::MatrixXd particular_solution = jacobianPseudoInverse * inCartVelocities;
    Eigen::MatrixXd homogeneous_solution = Eigen::MatrixXd::Zero(particular_solution.rows(), particular_solution.cols());
    Eigen::MatrixXd prediction_solution = Eigen::MatrixXd::Zero(particular_solution.rows(), 1);

    ROS_INFO_STREAM("===== task output =======");
    for (std::set<ConstraintBase_t>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        ROS_INFO_STREAM("task id: " << (*it)->getTaskId());
        (*it)->update(joint_states, prediction_solution, this->jacobian_data_);
        Eigen::VectorXd q_dot_0 = (*it)->getPartialValues();
        Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
        activation_gain = (*it)->getActivationGain(); // contribution of the homo. solution to the part. solution
        double constraint_k_H = (*it)->getSelfMotionMagnitude(particular_solution, tmpHomogeneousSolution); // gain of homogenous solution (if active)

        ROS_INFO_STREAM("activation_gain: " << activation_gain);
        ROS_INFO_STREAM("smm: " << constraint_k_H);

        homogeneous_solution += (constraint_k_H * activation_gain * tmpHomogeneousSolution);
    }

    Eigen::MatrixXd qdots_out = particular_solution + this->params_.k_H * homogeneous_solution; // weighting with k_H is done in loop
    return qdots_out;
}
