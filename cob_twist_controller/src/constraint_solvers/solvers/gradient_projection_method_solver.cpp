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
Eigen::MatrixXd GradientProjectionMethodSolver::solve(const t_Vector6d &inCartVelocities,
                                                      const JointStates& joint_states) const
{
    //static double last_distance = 0.0;
    //static double delta_distance_factor = 1.0;
    //static bool activated = false;
    double k_H;
    double min_dist;
    double activation;
    Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(joint_states.current_q_.rows());
    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
    Eigen::MatrixXd partialSolution = jacobianPseudoInverse * inCartVelocities;
    Eigen::MatrixXd homogeneousSolution = Eigen::MatrixXd::Zero(partialSolution.rows(), partialSolution.cols());

    for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        q_dot_0 = (*it)->getPartialValues();
        min_dist = (*it)->getValue();
        activation = (*it)->getActivationThreshold();
        Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
        k_H = (*it)->getSelfMotionMagnitude(partialSolution, tmpHomogeneousSolution);
        homogeneousSolution += (k_H * tmpHomogeneousSolution);
    }

    double smm_gain = SelfMotionMagnitudeFactory< SmmDeterminatorVelocityBounds<MIN_CRIT> >::calculate(this->params_,
                                                                                                       partialSolution,
                                                                                                       homogeneousSolution);
    homogeneousSolution = smm_gain * homogeneousSolution;

    if(activation > min_dist)
    {
        ROS_INFO_STREAM("min_dist: " << min_dist);
//        ROS_INFO_STREAM("delta_distance_factor: " << delta_distance_factor);
        ROS_INFO_STREAM("k_H: " << k_H);
        ROS_INFO_STREAM("smm gain: " << smm_gain);
//        ROS_INFO_STREAM("projector: " << std::endl << projector);
        ROS_INFO_STREAM("q_dot_0: " << std::endl << q_dot_0.transpose());
//        ROS_INFO_STREAM("partialSolution: " << std::endl << partialSolution);
//        ROS_INFO_STREAM("homogeneousSolution: " << std::endl << homogeneousSolution);
    }

//    last_distance = min_dist;
    Eigen::MatrixXd qdots_out = partialSolution + homogeneousSolution; // weighting with k_H is done in loop
    return qdots_out;
}
