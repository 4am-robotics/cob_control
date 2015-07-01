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
#include "cob_twist_controller/constraint_solvers/solvers/stack_of_tasks_gpm_solver.h"

#include <sstream>
#include <Eigen/Dense>

#include "cob_twist_controller/task_stack/task_stack_controller.h"



#include "cob_obstacle_distance/PredictDistance.h"

#include "cob_twist_controller/constraints/self_motion_magnitude.h"

/*
 * Split all cart velocities into separate tasks.
 * If conflicted with GPM then remove the task from processing.
 * With a prediction the deactivated task will be reactivated again.
 */
Eigen::MatrixXd StackOfTasksSolver2nd::solve(const t_Vector6d& in_cart_velocities,
                                             const JointStates& joint_states)
{
    double magnitude;
    double V_q;
    double crit_distance;
    double min_dist;
    double activation_gain;
    double cycle_time = 0.0;
    double now_time = ros::Time::now().toSec();
    Eigen::VectorXd q_i = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    Eigen::MatrixXd projector_i = Eigen::MatrixXd::Identity(this->jacobian_data_.cols(), this->jacobian_data_.cols());

    std::string task_to_ignore = "";
    double crit = 9999999;
    double crit_scalar = 0.0;


    ROS_INFO_STREAM("last_cycle_time_: " << last_cycle_time_);


    if(last_cycle_time_ > 0.0)
    {
        cycle_time = now_time - last_cycle_time_;
    }

    Eigen::VectorXd eigen_vec_last_q_dot = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    for (int i = 0; i < joint_states.current_q_dot_.rows(); ++i)
    {
        eigen_vec_last_q_dot(i) = joint_states.current_q_dot_(i);
    }

    Eigen::VectorXd eigen_vec_last_q = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    for (int i = 0; i < joint_states.current_q_.rows(); ++i)
    {
        eigen_vec_last_q(i) = joint_states.current_q_(i);
    }

    t_Vector6d tmp_in_cart_velocities = in_cart_velocities;
    Eigen::VectorXd sum_of_gradient = Eigen::VectorXd::Zero(joint_states.current_q_.rows());
    // Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(joint_states.current_q_.rows());
    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
    Eigen::VectorXd partialSolution = jacobianPseudoInverse * in_cart_velocities;
    Eigen::MatrixXd homogeneousSolution = Eigen::MatrixXd::Zero(partialSolution.rows(), partialSolution.cols());

    for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        (*it)->update(joint_states);
        activation_gain = (*it)->getActivationGain();
        min_dist = (*it)->getCriticalValue();
        crit_distance = (*it)->getActivationThreshold();
        Eigen::VectorXd q_dot_0 = params_.k_H * activation_gain * (*it)->getPartialValues();
        Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
        magnitude = (*it)->getSelfMotionMagnitude(partialSolution, tmpHomogeneousSolution);
        sum_of_gradient += magnitude * q_dot_0; // smm adapted q_dot_0 vector
        homogeneousSolution += magnitude * tmpHomogeneousSolution; // smm adapted homo. solution
        V_q = (*it)->getValue();
    }

    TaskStackController_t* tsc = this->params_.task_stack_controller;
    for(int32_t taskNr = 0; taskNr < in_cart_velocities.rows(); ++taskNr) // TODO: where to get max number of tasks?
    {
        Eigen::MatrixXd J_task = this->jacobian_data_.row(taskNr);
        Eigen::VectorXd task = in_cart_velocities.row(taskNr);
        std::ostringstream oss;
        oss << taskNr;
        Task_t t(taskNr, oss.str(), J_task, task);
        tsc->addTask(t);

        Eigen::MatrixXd J_task_inv = pinv_calc_.calculate(this->params_, this->damping_, J_task);
        Eigen::MatrixXd projector_task = Eigen::MatrixXd::Identity(J_task_inv.rows(), J_task.cols()) - J_task_inv * J_task;
        Eigen::VectorXd pg = projector_task * sum_of_gradient;
        Eigen::VectorXd left = J_task_inv * in_cart_velocities.row(taskNr);

        if (pg.norm() > 0.0 && crit > pg.norm()) // && scalar < crit_scalar)
        {
            crit = pg.norm();
            task_to_ignore = oss.str();
            ROS_INFO_STREAM("Ignore task: " << task_to_ignore);
        }
    }

    /*
     * Prediction of distance:
     */

    double predicted_distance = -1.0;
    double min_predicted_distance = 999999.9;
    if (cycle_time > 0.0)
    {
        if (ros::service::exists("obstacle_distance/predictDistance", true))
        {
            cob_obstacle_distance::PredictDistance pd;
            pd.request.frame_id.push_back("arm_right_3_link");
            pd.request.frame_id.push_back("arm_right_5_link");
            pd.request.frame_id.push_back("arm_right_7_link");

            Eigen::MatrixXd predict_qdots_out = partialSolution + homogeneousSolution;
            Eigen::VectorXd new_eigen_vec_last_q = eigen_vec_last_q + cycle_time * predict_qdots_out;

            for(uint32_t i = 0; i < new_eigen_vec_last_q.rows(); ++i)
            {
                pd.request.joint_pos.push_back(static_cast<double>(new_eigen_vec_last_q(i)));
            }

            bool found = ros::service::call("obstacle_distance/predictDistance", pd);
            if(found)
            {
                for(uint32_t dist_idx = 0; dist_idx < pd.response.min_distances.size(); ++dist_idx)
                {
                    predicted_distance = static_cast<double>(pd.response.min_distances.at(dist_idx));

                    if(min_predicted_distance < 0.0)
                    {
                        min_predicted_distance = predicted_distance;
                    }
                    else if(predicted_distance < min_predicted_distance)
                    {
                        min_predicted_distance = predicted_distance;
                    }
                }
            }
        }
        else
        {
            ROS_ERROR("Service obstacle_distance/predictDistance does not exist.");
        }
    }
    else
    {
        ROS_ERROR("Cannot call service because cycle time is <= 0.0!!!");
    }

    predicted_distance = min_predicted_distance;


    if (task_to_ignore != "" &&
            predicted_distance > -1 && predicted_distance < (crit_distance * 1.1))
    {
        ROS_WARN_STREAM("Deactivation of task: " << task_to_ignore);
        tsc->deactivateTask(task_to_ignore);
    }

    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(this->jacobian_data_.cols(), 1);

    if(predicted_distance >= crit_distance)
    {
        tsc->activateAllTasks(); // in safe region activate all tasks again.
    }


    //if(min_dist >= 0.015)
    if(min_dist >= 0.001)
    {
        unsigned int lv = 0;
        TaskSetIter_t it = tsc->beginTaskIter();
        while((it = tsc->nextActiveTask()) != tsc->getTasksEnd())
        {
            Eigen::MatrixXd J_task = it->task_jacobian_;
            Eigen::MatrixXd J_temp = J_task * projector_i;
            Eigen::MatrixXd J_temp_inv = pinv_calc_.calculate(this->params_, this->damping_, J_temp);
            Eigen::VectorXd v_task = it->task_;
            q_i = q_i + J_temp_inv * (v_task - J_task * q_i);
            projector_i = projector_i - J_temp_inv * J_temp;
            lv ++;
        }

        ROS_INFO_STREAM("Active tasks: " << lv);
        qdots_out.col(0) = q_i + exp(params_.mu) * last_jac_ * last_in_cart_velocities_ + projector_i * sum_of_gradient;
    }
    else
    {
        ROS_ERROR_STREAM(">>>>>>>>>>>>>>>> Deactivation of task AND constrain -> STOP ALL MOTIONS !!!");
    }

    last_in_cart_velocities_ = in_cart_velocities;
    last_jac_ = jacobianPseudoInverse;
    last_min_distance_ = min_dist;
    last_cycle_time_ = now_time;
    return qdots_out;
}


