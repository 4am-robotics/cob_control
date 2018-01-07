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

#include "cob_twist_controller/constraint_solvers/solvers/stack_of_tasks_solver.h"
#include "cob_twist_controller/task_stack/task_stack_controller.h"

Eigen::MatrixXd StackOfTasksSolver::solve(const Vector6d_t& in_cart_velocities,
                                          const JointStates& joint_states)
{
    this->global_constraint_state_ = NORMAL;
    ros::Time now = ros::Time::now();
    double cycle = (now - this->last_time_).toSec();
    this->last_time_ = now;

    Eigen::MatrixXd damped_pinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd pinv = pinv_calc_.calculate(this->jacobian_data_);

    Eigen::MatrixXd particular_solution = damped_pinv * in_cart_velocities;

    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(pinv.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - pinv * this->jacobian_data_;

    Eigen::MatrixXd projector_i = Eigen::MatrixXd::Identity(this->jacobian_data_.cols(), this->jacobian_data_.cols());
    Eigen::VectorXd q_i = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(this->jacobian_data_.cols(), 1);

    Eigen::VectorXd sum_of_gradient = Eigen::VectorXd::Zero(this->jacobian_data_.cols());

    KDL::JntArrayVel predict_jnts_vel(joint_states.current_q_.rows());

    // predict next joint states!
    for (int i = 0; i < joint_states.current_q_.rows(); ++i)
    {
        predict_jnts_vel.q(i) = particular_solution(i, 0) * cycle + joint_states.current_q_(i);
        predict_jnts_vel.qdot(i) = particular_solution(i, 0);
    }

    // First iteration: update constraint state and calculate the according GPM weighting (DANGER state)
    double inv_sum_of_prionums = 0.0;
    for (std::set<ConstraintBase_t>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        (*it)->update(joint_states, predict_jnts_vel, this->jacobian_data_);
        const double constr_prio = (*it)->getPriorityAsNum();
        if ((*it)->getState().getCurrent() == DANGER)
        {
            inv_sum_of_prionums += constr_prio > ZERO_THRESHOLD ? 1.0 / constr_prio : 1.0 / DIV0_SAFE;
        }
    }

    // Second iteration: Process constraints with sum of prios for active GPM constraints!
    for (std::set<ConstraintBase_t>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        this->processState(it, projector, particular_solution, inv_sum_of_prionums, sum_of_gradient);
    }

    sum_of_gradient = this->params_.k_H * sum_of_gradient;  // "global" weighting for all constraints.

    if (CRITICAL == this->global_constraint_state_)
    {
        this->in_cart_vel_damping_ = START_CNT;
    }
    else
    {
        this->in_cart_vel_damping_ = this->in_cart_vel_damping_ > 1.0 ? (this->in_cart_vel_damping_ - 1.0) : 1.0;
    }

    const Vector6d_t scaled_in_cart_velocities = (1.0 / pow(this->in_cart_vel_damping_, 2.0)) * in_cart_velocities;
    Task_t t(this->params_.priority_main, "Main task", this->jacobian_data_, scaled_in_cart_velocities);
    t.tcp_ = this->params_;
    this->task_stack_controller_.addTask(t);

    // ROS_INFO_STREAM("============== Task output ============= with main task damping: " << this->in_cart_vel_damping_);
    TaskSetIter_t it = this->task_stack_controller_.beginTaskIter();
    while ((it = this->task_stack_controller_.nextActiveTask()) != this->task_stack_controller_.getTasksEnd())
    {
        Eigen::MatrixXd J_task = it->task_jacobian_;
        Eigen::MatrixXd J_temp = J_task * projector_i;
        Eigen::VectorXd v_task = it->task_;
        Eigen::MatrixXd J_temp_inv = pinv_calc_.calculate(J_temp);  //ToDo: Do we need damping here?
        q_i = q_i + J_temp_inv * (v_task - J_task * q_i);
        projector_i = projector_i - J_temp_inv * J_temp;
    }

    qdots_out.col(0) = q_i + projector_i * sum_of_gradient;
    return qdots_out;
}


void StackOfTasksSolver::processState(std::set<ConstraintBase_t>::iterator& it,
                                      const Eigen::MatrixXd& projector,
                                      const Eigen::MatrixXd& particular_solution,
                                      double inv_sum_of_prios,
                                      Eigen::VectorXd& sum_of_gradient)
{
    Eigen::VectorXd q_dot_0 = (*it)->getPartialValues();
    const double activation_gain = (*it)->getActivationGain();
    Eigen::MatrixXd tmp_projection = projector * q_dot_0;
    const double magnitude = (*it)->getSelfMotionMagnitude(particular_solution, tmp_projection);
    ConstraintState cstate = (*it)->getState();

    const double constr_prio = (*it)->getPriorityAsNum();
    const double inv_constr_prio = constr_prio > ZERO_THRESHOLD ? 1.0 / constr_prio : 1.0 / DIV0_SAFE;
    // only necessary for GPM sum because task stack is already sorted according to PRIOs.
    const double gpm_weighting = inv_constr_prio / inv_sum_of_prios;

    if (cstate.isTransition())
    {
        if (cstate.getCurrent() == CRITICAL)
        {
            // "global" weighting k_H for all constraint tasks.
            Task_t t = (*it)->createTask();
            double factor = activation_gain * std::abs(magnitude);
            t.task_ = factor * t.task_;
            this->task_stack_controller_.addTask(t);
            this->task_stack_controller_.activateTask((*it)->getTaskId());
        }
        else if (cstate.getCurrent() == DANGER)
        {
            this->task_stack_controller_.deactivateTask((*it)->getTaskId());
            sum_of_gradient += gpm_weighting * activation_gain * magnitude * q_dot_0;  // smm adapted q_dot_0 vector
        }
        else
        {
            this->task_stack_controller_.deactivateTask((*it)->getTaskId());
        }
    }
    else
    {
        if (cstate.getCurrent() == CRITICAL)
        {
            Task_t t = (*it)->createTask();
            double factor = activation_gain * std::abs(magnitude);  // task must be decided whether negative or not!
            t.task_ = factor * t.task_;
            this->task_stack_controller_.addTask(t);
        }
        else if (cstate.getCurrent() == DANGER)
        {
            sum_of_gradient += gpm_weighting * activation_gain * magnitude * q_dot_0;  // smm adapted q_dot_0 vector
        }
        else
        {
            // just compute the particular solution
        }
    }

    if (cstate.getCurrent() > this->global_constraint_state_ )
    {
        this->global_constraint_state_ = cstate.getCurrent();
    }
}
