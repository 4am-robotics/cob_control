/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <set>

#include <cob_twist_controller/constraint_solvers/solvers/task_priority_solver.h>

/**
 * Solve the inverse differential kinematics equation by using a two tasks.
 * Maciejewski A., Obstacle Avoidance for Kinematically Redundant Manipulators in Dyn Varying Environments.
 */
Eigen::MatrixXd TaskPrioritySolver::solve(const Vector6d_t& in_cart_velocities,
                                          const JointStates& joint_states)
{
    ros::Time now = ros::Time::now();
    double cycle = (now - this->last_time_).toSec();
    this->last_time_ = now;

    double derivative_cost_func_value;
    double current_cost_func_value;
    double activation_gain;
    double magnitude;

    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(this->jacobian_data_.cols(), 1);
    Eigen::VectorXd partial_cost_func = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    Eigen::MatrixXd damped_pinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd pinv = pinv_calc_.calculate(this->jacobian_data_);

    Eigen::MatrixXd particular_solution = damped_pinv * in_cart_velocities;

    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(pinv.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - pinv * this->jacobian_data_;

    KDL::JntArrayVel predict_jnts_vel(joint_states.current_q_.rows());

    // predict next joint states!
    for (uint8_t i = 0; i < joint_states.current_q_.rows(); ++i)
    {
        predict_jnts_vel.q(i) = particular_solution(i, 0) * cycle + joint_states.current_q_(i);
        predict_jnts_vel.qdot(i) = particular_solution(i, 0);
    }

    if (this->constraints_.size() > 0)
    {
        for (std::set<ConstraintBase_t>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
        {
            (*it)->update(joint_states, predict_jnts_vel, this->jacobian_data_);
            current_cost_func_value = (*it)->getValue();
            derivative_cost_func_value = (*it)->getDerivativeValue();
            partial_cost_func = (*it)->getPartialValues();  // Equal to (partial g) / (partial q) = J_g
            activation_gain = (*it)->getActivationGain();
            magnitude = (*it)->getSelfMotionMagnitude(Eigen::MatrixXd::Zero(1, 1), Eigen::MatrixXd::Zero(1, 1));  // not necessary to pass valid values here.

            ROS_INFO_STREAM("activation_gain: " << activation_gain);
            ROS_INFO_STREAM("smm: " << magnitude);
        }

        Eigen::MatrixXd jac_inv_2nd_term = Eigen::MatrixXd::Zero(projector.cols(), partial_cost_func.cols());
        if (activation_gain > 0.0)
        {
            Eigen::MatrixXd tmp_matrix = partial_cost_func.transpose() * projector;
            jac_inv_2nd_term = pinv_calc_.calculate(tmp_matrix);
        }

        Eigen::MatrixXd m_derivative_cost_func_value = derivative_cost_func_value * Eigen::MatrixXd::Identity(1, 1);
        qdots_out = particular_solution + this->params_.k_H * activation_gain * jac_inv_2nd_term * (magnitude * m_derivative_cost_func_value - partial_cost_func.transpose() * particular_solution);
    }
    else
    {
        qdots_out = particular_solution;
        ROS_ERROR_STREAM("Should not occur solution: " << std::endl << qdots_out);
    }

    // Eigen::MatrixXd qdots_out = particular_solution + homogeneousSolution; // weighting with k_H is done in loop
    return qdots_out;
}

