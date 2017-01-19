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
	Eigen::MatrixXd pinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
	Eigen::VectorXd q_dot = pinv * in_cart_velocities;
	Eigen::MatrixXd W_WLN = this->calculateWeighting(q_dot, joint_states);
/*
	Eigen::MatrixXd J = pinv_calc_.calculate(pinv);
	Eigen::MatrixXd Jt = J.transpose() ;
	Eigen::MatrixXd inv_W_WLN = pinv_calc_.calculate(J*W_WLN*Jt);
	Eigen::MatrixXd qdots_out =  W_WLN * Jt * inv_W_WLN * in_cart_velocities;*/

    Eigen::MatrixXd Jw =W_WLN * pinv;
    ROS_INFO_STREAM("wEIGTH:"<<W_WLN(1,1));
    Eigen::MatrixXd qdots_out2 = pinv * in_cart_velocities;
    ROS_INFO_STREAM("Joint Velocity no JLA:"<<qdots_out2(1));
    Eigen::MatrixXd qdots_out =  Jw* in_cart_velocities;
    ROS_INFO_STREAM("Joint Velocity:"<<qdots_out(1));
    return qdots_out;
}

/**
 * This function returns the identity as weighting matrix for base functionality.
 */
Eigen::MatrixXd WeightedLeastNormSigmoidSolver::calculateWeighting(Eigen::VectorXd q_dot, const JointStates& joint_states) const
{
    std::vector<double> limits_min = this->limiter_params_.limits_min;
    std::vector<double> limits_max = this->limiter_params_.limits_max;
    uint32_t cols = this->jacobian_data_.cols();

    Eigen::VectorXd weighting = Eigen::VectorXd::Zero(cols);

    KDL::JntArray q = joint_states.current_q_;

    double sigma = this->params_.damping_jla;
    double sigma_speed = this->params_.damping_speed_jla;
    double delta_pos = this->params_.thresholds_jla.activation_position_threshold_jla;
	double delta_speed = this->params_.thresholds_jla.activation_speed_threshold_jla;
    for (uint32_t i = 0; i < cols ; ++i)
    {

      weighting(i) = (1.0/(1.0+exp(-(q(i)-limits_min[i]-delta_pos)/sigma)))*(1.0/(1.0+exp((q(i)-limits_max[i]+delta_pos)/sigma)))+(1.0/(1.0+exp((q(i)*q_dot(i)+delta_speed)*sigma_speed)));

      if (weighting(i) > 1.0){
    	  weighting(i)=1.0;
      }
    }

    return weighting.asDiagonal();
}

