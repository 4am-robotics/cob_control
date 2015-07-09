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
 * \date Date of creation: June, 2015
 *
 * \brief
 *   Implementation of a priority based solver for several tasks
 *   with additional gradient projection.
 *
 ****************************************************************/
#include "cob_twist_controller/constraint_solvers/solvers/dynamic_tasks_readjust_solver.h"
#include "cob_twist_controller/task_stack/task_stack_controller.h"
#include "cob_twist_controller/constraints/self_motion_magnitude.h"

Eigen::MatrixXd DynamicTasksReadjustSolver::solve(const t_Vector6d& in_cart_velocities,
                                                  const JointStates& joint_states)
{
    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
    Eigen::MatrixXd particular_solution = jacobianPseudoInverse * in_cart_velocities;

    Eigen::MatrixXd joint_pos = Eigen::MatrixXd::Zero(particular_solution.rows(), 1);
    Eigen::MatrixXd prediction_solution = Eigen::MatrixXd::Zero(particular_solution.rows(), 1);
    Eigen::MatrixXd projector_i = Eigen::MatrixXd::Identity(this->jacobian_data_.cols(), this->jacobian_data_.cols());
    Eigen::VectorXd q_i = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(this->jacobian_data_.cols(), 1);

    Eigen::VectorXd sum_of_gradient = Eigen::VectorXd::Zero(this->jacobian_data_.cols());

    ROS_INFO_STREAM("============== Constraint output =============");
    for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        ROS_INFO_STREAM("constraint id: " << (*it)->getTaskId());
        (*it)->update(joint_states, prediction_solution, this->jacobian_data_);
        this->processState(it, projector, particular_solution, sum_of_gradient);
    }

    sum_of_gradient = this->params_.k_H * sum_of_gradient; // "global" weighting for all constraints.

    Task_t t(this->params_.priority_main, "Main task", this->jacobian_data_, in_cart_velocities);
    t.tcp_ = this->params_;
    t.db_ = this->damping_;
    this->task_stack_controller_.addTask(t);

    ROS_INFO_STREAM("============== Task output =============");
    TaskSetIter_t it = this->task_stack_controller_.beginTaskIter();
    while((it = this->task_stack_controller_.nextActiveTask()) != this->task_stack_controller_.getTasksEnd())
    {
        ROS_INFO_STREAM("id: " << it->id_);
        Eigen::MatrixXd J_task = it->task_jacobian_;
        Eigen::MatrixXd J_temp = J_task * projector_i;

        Eigen::MatrixXd J_temp_inv;
        Eigen::VectorXd v_task = it->task_;

        J_temp_inv = pinv_calc_.calculate(it->tcp_, it->db_, J_temp);
        q_i = q_i + J_temp_inv * (v_task - J_task * q_i);
        projector_i = projector_i - J_temp_inv * J_temp;

        ROS_INFO_STREAM("q_i: " << q_i.transpose());
        // it->db_.reset();
    }

    qdots_out.col(0) = q_i + projector_i * sum_of_gradient;
    return qdots_out;
}


void DynamicTasksReadjustSolver::processState(std::set<tConstraintBase>::iterator& it,
                                              const Eigen::MatrixXd& projector,
                                              const Eigen::MatrixXd& particular_solution,
                                              Eigen::VectorXd& sum_of_gradient)
{
    Eigen::VectorXd q_dot_0 = (*it)->getPartialValues();
    double activation_gain = (*it)->getActivationGain();
    Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
    double magnitude = (*it)->getSelfMotionMagnitude(particular_solution, tmpHomogeneousSolution);
    ConstraintState cstate = (*it)->getState();
    if(cstate.isTransition())
    {
        if(cstate.getCurrent() == CRITICAL)
        {
            // Eigen::MatrixXd J_task0 = q_dot_0.transpose();
            // "global" weighting k_H for all constraint tasks.
            //Eigen::VectorXd task = this->params_.k_H * activation_gain * magnitude * derivative_value * Eigen::VectorXd::Identity(1, 1);
            Task_t t = (*it)->createTask();
            double factor = this->params_.k_H * activation_gain * magnitude;
            t.task_ = factor * t.task_;
            this->task_stack_controller_.addTask(t);
            this->task_stack_controller_.activateTask((*it)->getTaskId());

        }
        else if(cstate.getCurrent() == DANGER)
        {
            ROS_WARN_STREAM("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
            ROS_WARN_STREAM("-> DANGER!!!");
            this->task_stack_controller_.deactivateTask((*it)->getTaskId());
            sum_of_gradient += activation_gain * magnitude * q_dot_0; // smm adapted q_dot_0 vector
        }
        else
        {
            this->task_stack_controller_.deactivateTask((*it)->getTaskId());
        }
    }
    else
    {
        if(cstate.getCurrent() == CRITICAL)
        {
            // Eigen::MatrixXd J_task0 = q_dot_0.transpose();
            // "global" weighting k_H for all constraint tasks.
            //Eigen::VectorXd task = this->params_.k_H * activation_gain * magnitude * derivative_value * Eigen::VectorXd::Identity(1, 1);
            Task_t t = (*it)->createTask();
            // double factor = this->params_.k_H * activation_gain * std::abs(magnitude);
            double factor = this->params_.k_H * activation_gain * magnitude;
            t.task_ = factor * t.task_;

            // t.task_jacobian_ = factor * t.task_jacobian_;
            this->task_stack_controller_.addTask(t);
        }
        else if(cstate.getCurrent() == DANGER)
        {
            ROS_WARN_STREAM("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
            ROS_WARN_STREAM("DANGER!!!");
            sum_of_gradient += activation_gain * magnitude * q_dot_0; // smm adapted q_dot_0 vector
        }
        else
        {
            // just compute the particular solution
        }
    }
}
