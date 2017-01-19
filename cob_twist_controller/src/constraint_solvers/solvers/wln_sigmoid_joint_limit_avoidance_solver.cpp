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

#include <vector>
#include <ros/ros.h>

#include "cob_twist_controller/constraint_solvers/solvers/wln_sigmoid_joint_limit_avoidance_solver.h"

/**
 * This function calculates the weighting matrix used to penalize a joint when it is near and moving towards a limit.
 * The last joint velocity is used to determine if it that happens or not
 */
Eigen::MatrixXd WLN_Sigmoid_JointLimitAvoidanceSolver::calculateWeighting(const Vector6d_t& in_cart_velocities, const JointStates& joint_states) const
{
	Eigen::MatrixXd Jinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
	Eigen::VectorXd q_dot = Jinv * in_cart_velocities;
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

	    weighting(i) = 1.0/((1.0/(1.0+exp(-(q(i)-limits_min[i]-delta_pos)/sigma)))*(1.0/(1.0+exp((q(i)-limits_max[i]+delta_pos)/sigma)))+(1.0/(1.0+exp((q(i)*q_dot(i)+delta_speed)*sigma_speed))));

    }

    return weighting.asDiagonal();
}
