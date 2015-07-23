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

Eigen::MatrixXd DynamicTasksReadjustSolver::solve(const Vector6d_t& in_cart_velocities,
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

    ros::Time last_ts_mod_time = this->task_stack_controller_.getLastModificationTime();

    for (std::set<ConstraintBase_t>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        (*it)->update(joint_states, prediction_solution, this->jacobian_data_);
        this->processState(it, projector, particular_solution, sum_of_gradient);
    }

    sum_of_gradient = this->params_.k_H * sum_of_gradient; // "global" weighting for all constraints.

    //const Vector6d_t scaled_in_cart_velocities = (1.0 / pow(this->in_cart_vel_damping_, 2.0)) * in_cart_velocities;

    //const Vector6d_t scaled_in_cart_velocities = (1.0 - this->in_cart_vel_damping_) * in_cart_velocities;
    const Vector6d_t scaled_in_cart_velocities = in_cart_velocities;

    Task_t t(this->params_.priority_main, "Main task", this->jacobian_data_, scaled_in_cart_velocities);
    t.tcp_ = this->params_;
    t.db_ = this->damping_;
    this->task_stack_controller_.addTask(t);


///////////////////////////////////
    uint32_t cnt_task_rows = 0;
    TaskSetIter_t tmp_it = this->task_stack_controller_.beginTaskIter();
    while((tmp_it = this->task_stack_controller_.nextActiveTask()) != this->task_stack_controller_.getTasksEnd())
    {
        cnt_task_rows += tmp_it->task_.rows();
    }


    Eigen::MatrixXd J_full(cnt_task_rows, this->jacobian_data_.cols());
    Eigen::VectorXd task_full(cnt_task_rows, 1);
///////////////////////////////////

    ROS_INFO_STREAM("============== Task output ============= with main task damping: " << this->in_cart_vel_damping_);



    uint32_t block_start_row = 0;
    int active_tasks = this->task_stack_controller_.countActiveTasks();

    TaskSetIter_t it = this->task_stack_controller_.beginTaskIter();
    while((it = this->task_stack_controller_.nextActiveTask()) != this->task_stack_controller_.getTasksEnd())
    {
        ROS_INFO_STREAM("id: " << it->id_);
        Eigen::MatrixXd J_task = it->task_jacobian_;
        Eigen::MatrixXd J_temp = J_task * projector_i;

        Eigen::VectorXd v_task = it->task_;

        ///////////////////////////////////
        J_full.block(block_start_row, 0, J_temp.rows(), J_temp.cols()) = J_temp;
        task_full.block(block_start_row, 0, v_task.rows(), 1) = v_task;
        block_start_row += J_temp.rows();
        ///////////////////////////////////

        Eigen::MatrixXd J_temp_inv = pinv_calc_.calculate(it->tcp_, it->db_, J_temp);
        q_i = q_i + J_temp_inv * (v_task - J_task * q_i);
        projector_i = projector_i - J_temp_inv * J_temp;
    }

    //this->J_full_=  pinv_calc_.calculate(this->params_, this->damping_, J_full);

    Eigen::VectorXd last_mod_solution = Eigen::VectorXd::Zero(this->jacobian_data_.cols());

    //if(!this->init_ && this->last_active_tasks_ != active_tasks)
    if(!this->init_ && this->last_active_tasks_ != active_tasks)
    {
//        if(this->last_active_tasks_ < active_tasks)
//        {
//            this->J_full_ = J_full;
//            this->task_full_ = task_full;
//        }
//        else
//        {
//            ros::Duration d = ros::Time::now() - this->last_mod_time_;
//
//            Eigen::MatrixXd J_full_inv =  pinv_calc_.calculate(this->params_, this->damping_, this->J_full_);
//
//            last_mod_solution = exp(this->params_.mu * d.toSec()) * J_full_inv * this->task_full_;
//            ROS_WARN_STREAM("Current q_n: " << std::endl << q_i);
//            ROS_WARN_STREAM("CAlc last_mod_solution: " << std::endl << last_mod_solution);
//        }

        ros::Duration d = ros::Time::now() - this->last_mod_time_;

        Eigen::MatrixXd J_full_inv =  pinv_calc_.calculate(this->params_, this->damping_, this->J_full_);

        last_mod_solution = exp(this->params_.mu * d.toSec()) * J_full_inv * this->task_full_;


        ROS_WARN_STREAM("Current q_n: " << std::endl << q_i);
        ROS_WARN_STREAM("CAlc last_mod_solution: " << std::endl << last_mod_solution);

        ROS_INFO_STREAM("J_full: " << J_full);
        this->J_full_ = J_full;
        this->task_full_ = task_full;
        this->last_mod_time_ = ros::Time::now();
    }

    if(this->init_)
    {
        this->J_full_ = J_full;
        this->task_full_ = task_full;
        this->init_ = false;
    }



    this->last_active_tasks_ = active_tasks;

    //qdots_out.col(0) = q_i + projector_i * sum_of_gradient;
    qdots_out.col(0) = q_i + last_mod_solution + projector_i * sum_of_gradient;
    return qdots_out;
}


void DynamicTasksReadjustSolver::processState(std::set<ConstraintBase_t>::iterator& it,
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
            //double factor = this->params_.k_H * activation_gain * std::abs(magnitude);
            double factor = activation_gain * std::abs(magnitude);
            t.task_ = factor * t.task_;
            this->task_stack_controller_.addTask(t);
            this->task_stack_controller_.activateTask((*it)->getTaskId());

            this->in_cart_vel_damping_ = START_CNT;

        }
        else if(cstate.getCurrent() == DANGER)
        {
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
            // double factor = this->params_.k_H * activation_gain * std::abs(magnitude); // task must be decided whether negative or not!
            double factor = activation_gain * std::abs(magnitude); // task must be decided whether negative or not!
            t.task_ = factor * t.task_;

            // t.task_jacobian_ = factor * t.task_jacobian_;
            this->task_stack_controller_.addTask(t);
            this->in_cart_vel_damping_ = START_CNT;
        }
        else if(cstate.getCurrent() == DANGER)
        {
            sum_of_gradient += activation_gain * magnitude * q_dot_0; // smm adapted q_dot_0 vector
        }
        else
        {
            this->in_cart_vel_damping_ = this->in_cart_vel_damping_ > 1.0 ? (this->in_cart_vel_damping_ - 1.0) : 1.0;

            // just compute the particular solution
        }
    }
}
