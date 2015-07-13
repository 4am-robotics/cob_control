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
#include "cob_twist_controller/constraint_solvers/solvers/stack_of_tasks_solver.h"

#include <sstream>
#include <Eigen/Dense>

#include "cob_twist_controller/task_stack/task_stack_controller.h"



#include "cob_obstacle_distance/PredictDistance.h"


#include "cob_twist_controller/constraints/self_motion_magnitude.h"

/*
 * Split all cart velocities into separate tasks
 *
 *
 *
 *
 *
 *
 */
Eigen::MatrixXd StackOfTasksSolver::solve(const Vector6d_t& in_cart_velocities,
                                          const JointStates& joint_states)
{
    double now_time = ros::Time::now().toSec();
    double magnitude;
    double activation_gain;
    double derivative_value = 0.0;

    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(this->jacobian_data_.cols(), 1);

    Vector6d_t tmp_in_cart_velocities = in_cart_velocities;
    Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
    Eigen::MatrixXd partialSolution = jacobianPseudoInverse * in_cart_velocities;
    Eigen::MatrixXd prediction_solution = Eigen::MatrixXd::Zero(partialSolution.rows(), 1);
    for(uint8_t i = 0; i < joint_states.current_q_.rows(); ++i)
    {
        prediction_solution(i, 0) = partialSolution(i, 0) * 0.02 + joint_states.current_q_(i);
    }

    for (std::set<ConstraintBase_t>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        (*it)->update(joint_states, prediction_solution, this->jacobian_data_);
        q_dot_0 = (*it)->getPartialValues();
        activation_gain = (*it)->getActivationGain();
        Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
        magnitude = (*it)->getSelfMotionMagnitude(partialSolution, tmpHomogeneousSolution);
        derivative_value = (*it)->getDerivativeValue();

        if(activation_gain > 0.0)
        {
            Eigen::MatrixXd J_task0 = q_dot_0.transpose();
            // "global" weighting for all constraints in params_.k_H
            Eigen::VectorXd task = this->params_.k_H * activation_gain * magnitude * derivative_value * Eigen::VectorXd::Identity(1, 1);
            Task_t t((*it)->getPriority(),(*it)->getTaskId() , J_task0, task);
            this->task_stack_controller_.addTask(t);
        }
    }

    Eigen::VectorXd q_i = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    Eigen::MatrixXd projector_i = Eigen::MatrixXd::Identity(this->jacobian_data_.cols(), this->jacobian_data_.cols());

    Task_t t(this->params_.priority_main, "Main task", this->jacobian_data_, in_cart_velocities);
    this->task_stack_controller_.addTask(t);

//    for(int32_t taskNr = 0; taskNr < 2; ++taskNr) // TODO: where to get max number of tasks? A POSITION AND A ORIENTATION TASK
//    {
//        Eigen::MatrixXd J_task = this->jacobian_data_.block(taskNr * 3, 0, 3, this->jacobian_data_.cols());
//        Eigen::VectorXd task = in_cart_velocities.block(taskNr * 3, 0, 3, 1);
//
//        std::ostringstream oss;
//        oss << taskNr;
//        Task_t t(taskNr, oss.str(), J_task, task);
//        tsc->addTask(t);
//    }

//    for(int32_t taskNr = 0; taskNr < in_cart_velocities.rows(); ++taskNr) // TODO: where to get max number of tasks?
//    {
//        Eigen::MatrixXd J_task = this->jacobian_data_.row(taskNr);
//        Eigen::VectorXd task = in_cart_velocities.row(taskNr);
//        std::ostringstream oss;
//        oss << taskNr;
//        Task_t t(MAIN_TASK_PRIO + taskNr, oss.str(), J_task, task);
//        tsc->addTask(t);
//    }

    TaskSetIter_t it = this->task_stack_controller_.beginTaskIter();
    while((it = this->task_stack_controller_.nextActiveTask()) != this->task_stack_controller_.getTasksEnd())
    {
        ROS_INFO_STREAM("Task: " << it->id_);
        Eigen::MatrixXd J_task = it->task_jacobian_;
        Eigen::MatrixXd J_temp = J_task * projector_i;
        Eigen::MatrixXd J_temp_inv = pinv_calc_.calculate(this->params_, this->damping_, J_temp);
        Eigen::VectorXd v_task = it->task_;
        q_i = q_i + J_temp_inv * (v_task - J_task * q_i);
        projector_i = projector_i - J_temp_inv * J_temp;
    }

    qdots_out.col(0) = q_i;
    return qdots_out;
}

