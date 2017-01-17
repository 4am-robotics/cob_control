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

#include "cob_twist_controller/constraint_solvers/solvers/wln_sigmoid_joint_limit_avoidance_solver.h"

/**
 * Specific implementation of the solve method using a weighted least norm.
 * This is done by calculation of a weighting which is dependent on inherited classes for the Jacobian.
 * Uses the base implementation of calculatePinvJacobianBySVD to calculate the pseudo-inverse (weighted) Jacobian.
 */
Eigen::MatrixXd WeightedLeastNormSigmoidSolver::solve(const Vector6d_t& in_cart_velocities,
                                               const JointStates& joint_states)
{
    Eigen::MatrixXd W_WLN = this->calculateWeighting(joint_states);

    Eigen::MatrixXd pinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);

    Eigen::MatrixXd J = pinv_calc_.calculate(pinv);
    // SVD of JLA weighted Jacobian: Damping will be done later in calculatePinvJacobianBySVD for pseudo-inverse Jacobian with additional truncation etc.
    Eigen::MatrixXd weighted_jacobian = W_WLN * J.transpose();

    Eigen::MatrixXd J_W_WLN =  J * W_WLN * J.transpose();

    Eigen::MatrixXd inv_W_WLN =  pinv_calc_.calculate(J_W_WLN);     // -> W*J'* (J * W * J')^-1

    // Take care: W^(1/2) * q_dot = weighted_pinv_J * x_dot -> One must consider the weighting!!!
    Eigen::MatrixXd qdots_out = weighted_jacobian * inv_W_WLN * in_cart_velocities;
    ROS_INFO_STREAM("Joint Velocity:"<<qdots_out(1));
    return qdots_out;
}

/**
 * This function returns the identity as weighting matrix for base functionality.
 */
Eigen::MatrixXd WeightedLeastNormSigmoidSolver::calculateWeighting(const JointStates& joint_states) const
{
    std::vector<double> limits_min = this->limiter_params_.limits_min;
    std::vector<double> limits_max = this->limiter_params_.limits_max;
    uint32_t cols = this->jacobian_data_.cols();
    Eigen::VectorXd weighting = Eigen::VectorXd::Zero(cols);

    KDL::JntArray q = joint_states.current_q_;
    KDL::JntArray q_dot = joint_states.current_q_dot_;
    double sigma = 0.05;
    for (uint32_t i = 0; i < cols ; ++i)
    {

      // calculation is only necessary in case of condition is true!
      weighting(i) = (1/(1+exp(-(q(i)-limits_min[i]-0.5)/sigma)))*(1/(1+exp((q(i)-limits_max[i]+0.5)/sigma)))+(1/(1+exp((q(i)*q_dot(i)+1)*5)));

      //DEBUG
      if(i==1){
      ROS_INFO_STREAM("Max. Limit:"<<limits_max[i]);
      ROS_INFO_STREAM("Min. Limit:"<<limits_min[i]);
      ROS_INFO_STREAM("Joint pos "<<i<<" :"<<q(i));
      ROS_INFO_STREAM("Weight:"<<weighting(i));

      }
    }

    return weighting.asDiagonal();
}

