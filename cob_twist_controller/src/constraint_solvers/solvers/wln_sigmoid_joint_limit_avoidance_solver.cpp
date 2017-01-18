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
	//INITIALIZE VARIABLES
	uint32_t cols = this->jacobian_data_.cols();
	uint32_t rows = this->jacobian_data_.rows();

	Eigen::MatrixXd W_WLN(Eigen::MatrixXd::Zero(cols,cols));
	Eigen::MatrixXd pinv(Eigen::MatrixXd::Zero(cols,rows));
	Eigen::MatrixXd J(Eigen::MatrixXd::Zero(rows,cols));
    W_WLN = this->calculateWeighting(in_cart_velocities, joint_states);

    pinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);

    J = pinv_calc_.calculate(pinv);
    // SVD of JLA weighted Jacobian: Damping will be done later in calculatePinvJacobianBySVD for pseudo-inverse Jacobian with additional truncation etc.
    Eigen::MatrixXd weighted_jacobian = W_WLN * J.transpose();

    Eigen::MatrixXd J_W_WLN =  J * W_WLN;

    Eigen::MatrixXd J_W_WLN_Jt =  J_W_WLN * J.transpose();

    Eigen::MatrixXd inv_W_WLN =  pinv_calc_.calculate(J_W_WLN_Jt);     // -> W*J'* (J * W * J')^-1

    // Take care: W^(1/2) * q_dot = weighted_pinv_J * x_dot -> One must consider the weighting!!!
    Eigen::MatrixXd Jw =weighted_jacobian * inv_W_WLN;
    Eigen::MatrixXd Jw2 =W_WLN * pinv;
    Eigen::MatrixXd qdots_out2 = pinv * in_cart_velocities;
    ROS_INFO_STREAM("Joint Velocity:"<<qdots_out2(1));
    Eigen::MatrixXd qdots_out =  Jw2* in_cart_velocities;
    //qdots_out(1)=qdots_out(1)*W_WLN(1,1);
    ROS_INFO_STREAM("Joint Velocity weighted:"<<qdots_out(1));

    return qdots_out;
}

/**
 * This function returns the identity as weighting matrix for base functionality.
 */
Eigen::MatrixXd WeightedLeastNormSigmoidSolver::calculateWeighting(const Vector6d_t& in_cart_velocities, const JointStates& joint_states) const
{
    std::vector<double> limits_min = this->limiter_params_.limits_min;
    std::vector<double> limits_max = this->limiter_params_.limits_max;
    uint32_t cols = this->jacobian_data_.cols();
    uint32_t rows = this->jacobian_data_.rows();
    Eigen::VectorXd weighting = Eigen::VectorXd::Zero(cols);
    Eigen::MatrixXd pinv(Eigen::MatrixXd::Zero(cols,rows));

    pinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    KDL::JntArray q = joint_states.current_q_;
    Eigen::VectorXd q_dot = pinv * in_cart_velocities;
    double sigma = this->params_.damping_jla;
    for (uint32_t i = 0; i < cols ; ++i)
    {

      // calculation is only necessary in case of condition is true!
      weighting(i) = (1.0/(1.0+exp(-(q(i)-limits_min[i]-this->limiter_params_.limits_tolerance)/sigma)))*(1.0/(1.0+exp((q(i)-limits_max[i]+this->limiter_params_.limits_tolerance)/sigma)))+(1.0/(1.0+exp((q(i)*q_dot(i)+1.0)*5.0)));
      if (weighting(i) >1.0){
    	  weighting(i) = 1.0;
      }
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

