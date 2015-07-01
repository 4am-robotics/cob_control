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
 *   Implementation of a solver for a stack of tasks
 *
 ****************************************************************/
#include <cob_twist_controller/constraint_solvers/solvers/task_priority_solver.h>

/**
 * Solve the inverse differential kinematics equation by using a two tasks.
 * Maciejewski A., Obstacle Avoidance for Kinematically Redundant Manipulators in Dyn Varying Environments.
 */
//
Eigen::MatrixXd TaskPrioritySolver::solve(const t_Vector6d& in_cart_velocities,
                                          const JointStates& joint_states)
{
    double derivative_cost_func_value;
    double current_cost_func_value;
    double activation_gain;
    double magnitude;

    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(joint_states.current_q_.rows(), 1);
    Eigen::VectorXd partial_cost_func = Eigen::VectorXd::Zero(joint_states.current_q_.rows());
    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
    Eigen::MatrixXd particular_solution = jacobianPseudoInverse * in_cart_velocities;

    if (this->constraints_.size() > 0)
    {
        for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
        {
            (*it)->update(joint_states);
            partial_cost_func = (*it)->getPartialValues(); // Equal to (partial g) / (partial q) = J_g
            current_cost_func_value = (*it)->getValue();
            derivative_cost_func_value = (*it)->getDerivativeValue();
            activation_gain = (*it)->getActivationGain();
            magnitude = (*it)->getSelfMotionMagnitude(Eigen::MatrixXd::Zero(1,1), Eigen::MatrixXd::Zero(1,1)); // not necessary to pass valid values here.

            ROS_INFO_STREAM("activation_gain: " << activation_gain);
            ROS_INFO_STREAM("smm: " << magnitude);
        }

        Eigen::MatrixXd jac_inv_2nd_term = Eigen::MatrixXd::Zero(projector.cols(), partial_cost_func.cols());
        if(activation_gain > 0.0)
        {
            Eigen::MatrixXd tmp_matrix = partial_cost_func.transpose() * projector;
            //boost::shared_ptr<DampingConstant> dbc(new DampingConstant(this->params_));
            jac_inv_2nd_term = pinv_calc_.calculate(this->params_, this->damping_, tmp_matrix);
            ROS_INFO_STREAM("jac_inv_2nd_term: " << jac_inv_2nd_term);
            //dbc.reset();
        }

        Eigen::MatrixXd m_derivative_cost_func_value = derivative_cost_func_value * Eigen::MatrixXd::Identity(1,1);
        qdots_out = particular_solution + activation_gain * jac_inv_2nd_term * (magnitude * m_derivative_cost_func_value - partial_cost_func.transpose() * particular_solution);
        ROS_INFO_STREAM("qdots_out: " << std::endl << qdots_out);
    }
    else
    {

        qdots_out = particular_solution;
        ROS_ERROR_STREAM("Should not occur solution: " << std::endl << qdots_out);
    }

//    Eigen::MatrixXd qdots_out = particular_solution + homogeneousSolution; // weighting with k_H is done in loop
    return qdots_out;
}

