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
    Eigen::MatrixXd partialSolution = jacobianPseudoInverse * in_cart_velocities;

    Eigen::MatrixXd projector_i = Eigen::MatrixXd::Identity(this->jacobian_data_.cols(), this->jacobian_data_.cols());
    Eigen::VectorXd q_i = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(this->jacobian_data_.cols(), 1);

    Eigen::VectorXd sum_of_gradient = Eigen::VectorXd::Zero(this->jacobian_data_.cols());

    ROS_INFO_STREAM("============== Constraint output =============");
    TaskStackController_t* tsc = this->params_.task_stack_controller;
    for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        ROS_INFO_STREAM("constraint id: " << (*it)->getTaskId());
        (*it)->update(joint_states, this->jacobian_data_);
        this->processState(it, projector, partialSolution, sum_of_gradient);
    }

    sum_of_gradient = this->params_.k_H * sum_of_gradient; // "global" weighting for all constraints.

    Task_t t(this->params_.priority_main, "Main task", this->jacobian_data_, in_cart_velocities);
    tsc->addTask(t);

    ROS_INFO_STREAM("============== Task output =============");
    TaskSetIter_t it = tsc->beginTaskIter();
    while((it = tsc->nextActiveTask()) != tsc->getTasksEnd())
    {
        ROS_INFO_STREAM("Task: " << it->id_);
        Eigen::MatrixXd J_task = it->task_jacobian_;
        Eigen::MatrixXd J_temp = J_task * projector_i;
        Eigen::MatrixXd J_temp_inv = pinv_calc_.calculate(this->params_, this->damping_, J_temp);
        Eigen::VectorXd v_task = it->task_;
        q_i = q_i + J_temp_inv * (v_task - J_task * q_i);
        projector_i = projector_i - J_temp_inv * J_temp;
    }

    qdots_out.col(0) = q_i + projector_i * sum_of_gradient;
    return qdots_out;
}


void DynamicTasksReadjustSolver::processState(std::set<tConstraintBase>::iterator& it,
                                              const Eigen::MatrixXd& projector,
                                              const Eigen::MatrixXd& particular_solution,
                                              Eigen::VectorXd& sum_of_gradient)
{
    TaskStackController_t* tsc = this->params_.task_stack_controller;
    Eigen::VectorXd q_dot_0 = (*it)->getPartialValues();

    double activation_gain = (*it)->getActivationGain();
    Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
    double magnitude = (*it)->getSelfMotionMagnitude(particular_solution, tmpHomogeneousSolution);
    double derivative_value = (*it)->getDerivativeValue();
    ConstraintState cstate = (*it)->getState();
    if(cstate.isTransition())
    {
        if(cstate.getCurrent() == CRITICAL)
        {
            Eigen::MatrixXd J_task0 = q_dot_0.transpose();
            // "global" weighting k_H for all constraint tasks.
            Eigen::VectorXd task = this->params_.k_H * activation_gain * magnitude * derivative_value * Eigen::VectorXd::Identity(1, 1);
            Task_t t((*it)->getPriority(), (*it)->getTaskId(), J_task0, task);
            tsc->addTask(t);
            tsc->activateTask((*it)->getTaskId());

        }
        else if(cstate.getCurrent() == DANGER)
        {
            tsc->deactivateTask((*it)->getTaskId());
            sum_of_gradient += activation_gain * magnitude * q_dot_0; // smm adapted q_dot_0 vector
        }
        else
        {
            tsc->deactivateTask((*it)->getTaskId());
        }
    }
    else
    {
        if(cstate.getCurrent() == CRITICAL)
        {
            Eigen::MatrixXd J_task0 = q_dot_0.transpose();
            // "global" weighting k_H for all constraint tasks.
            Eigen::VectorXd task = this->params_.k_H * activation_gain * magnitude * derivative_value * Eigen::VectorXd::Identity(1, 1);
            Task_t t((*it)->getPriority(), (*it)->getTaskId(), J_task0, task);
            tsc->addTask(t);
        }
        else if(cstate.getCurrent() == DANGER)
        {
            sum_of_gradient += activation_gain * magnitude * q_dot_0; // smm adapted q_dot_0 vector
        }
        else
        {
            // just compute the particular solution
        }
    }
}
