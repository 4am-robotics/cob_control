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
#include "cob_twist_controller/constraint_solvers/solvers/stack_of_tasks_solver_2nd.h"

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
Eigen::MatrixXd StackOfTasksSolver2nd::solve(const t_Vector6d &in_cart_velocities,
                                             const JointStates& joint_states) const
{
    static t_Vector6d last_in_cart_velocities = t_Vector6d::Zero();
    static t_Matrix76d last_jac = t_Matrix76d::Zero();

    static double last_min_distance = -1;

    static double last_integration_time = -1.0;
    double integration_time = 0.0;
    double now_integration_time = ros::Time::now().toSec();

    if(last_integration_time > 0.0)
    {
        integration_time = now_integration_time - last_integration_time;
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


    double current_time = ros::Time::now().toSec();

    ROS_INFO_STREAM("last_in_cart_velocities: " << last_in_cart_velocities.transpose());
    double k_H;
    double V_q;

    double crit_distance;
    double min_dist;

    t_Vector6d tmp_in_cart_velocities = in_cart_velocities;
    Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(joint_states.current_q_.rows());
    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
    Eigen::MatrixXd partialSolution = jacobianPseudoInverse * in_cart_velocities;
    Eigen::MatrixXd homogeneousSolution = Eigen::MatrixXd::Zero(partialSolution.rows(), partialSolution.cols());

    for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        q_dot_0 = (*it)->getPartialValues();
        crit_distance = (*it)->getActivationThreshold();
        min_dist = (*it)->getValue();
        Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
        k_H = (*it)->getSelfMotionMagnitude(partialSolution, tmpHomogeneousSolution);
        homogeneousSolution += (k_H * tmpHomogeneousSolution);

        V_q = (*it)->getValue();
    }

    Eigen::VectorXd q_i = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    Eigen::MatrixXd projector_i = Eigen::MatrixXd::Identity(this->jacobian_data_.cols(), this->jacobian_data_.cols());

    int32_t task_to_ignore = -1;
    double crit = 9999999;
    double crit_scalar = 0.0;


//    double smm_gain = SelfMotionMagnitudeFactory< SmmDeterminatorVelocityBounds<MIN_CRIT> >::calculate(this->params_,
//                                                                                                       partialSolution,
//                                                                                                       homogeneousSolution);


    TaskStackController_t *tsc = this->params_.task_stack_controller;


//    for(int32_t taskNr = 0; taskNr < 2; ++taskNr) // TODO: where to get max number of tasks? A POSITION AND A ORIENTATION TASK
//    {
//        Eigen::MatrixXd J_task = this->jacobian_data_.block(taskNr * 3, 0, 3, this->jacobian_data_.cols());
//        Eigen::VectorXd task = in_cart_velocities.block(taskNr * 3, 0, 3, 1);
//
//        std::ostringstream oss;
//        oss << taskNr;
//        Task_t t(taskNr, oss.str(), J_task, task);
//        tsc->addTask(t);
//        // ROS_INFO_STREAM("Task Jacobian: " << std::endl << J_task);
//
//        Eigen::MatrixXd J_task_inv = pinv_calc_.calculate(this->params_, this->damping_, J_task);
//        Eigen::MatrixXd projector_task = Eigen::MatrixXd::Identity(J_task_inv.rows(), J_task.cols()) - J_task_inv * J_task;
//        Eigen::VectorXd pg = projector_task * q_dot_0;
//
//        Eigen::VectorXd left = J_task_inv * in_cart_velocities.block(taskNr * 3, 0, 3, 1);
//        double scalar = left.dot(q_dot_0); // opposite direction?
//
//        if (q_dot_0.norm() > 0.0 && crit > pg.norm() && scalar < crit_scalar)
//        {
//            crit = pg.norm();
//            crit_scalar = scalar;
//            task_to_ignore = taskNr;
//        }
//    }

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

        Eigen::VectorXd pg = projector_task * q_dot_0;

        Eigen::VectorXd left = J_task_inv * in_cart_velocities.row(taskNr);
        double scalar = left.dot(q_dot_0); // opposite direction?

        if (q_dot_0.norm() > 0.0 && crit > pg.norm() && scalar < crit_scalar)
        {
            crit = pg.norm();
            task_to_ignore = taskNr;
            crit_scalar = scalar;
        }
    }

    /*
     * Prediction of distance:
     */

    double predicted_distance = -1.0;
    if (integration_time > 0.0)
    {
        if (ros::service::exists("obstacle_distance/predictDistance", true))
        {
            cob_obstacle_distance::PredictDistance pd;
            double magnitude = pow((2.0 * crit_distance / last_min_distance), 2.0) - 1.0;
            Eigen::MatrixXd predict_qdots_out = partialSolution + homogeneousSolution;

            Eigen::VectorXd new_eigen_vec_last_q = eigen_vec_last_q + integration_time * predict_qdots_out;

            for(uint32_t i = 0; i < new_eigen_vec_last_q.rows(); ++i)
            {
                pd.request.joint_pos.push_back(static_cast<double>(new_eigen_vec_last_q(i)));
            }

            pd.request.frame_id = "arm_right_5_link";
            bool found = ros::service::call("obstacle_distance/predictDistance", pd);
            if(found)
            {
                predicted_distance = static_cast<double>(pd.response.min_dist);

                ROS_INFO_STREAM("!!!!!!!!!!! Predicted distance: " << predicted_distance);
            }
        }
        else
        {
            ROS_ERROR("Service obstacle_distance/predictDistance does not exist.");
        }
    }



    if (task_to_ignore != -1 &&
            predicted_distance > -1 && predicted_distance < crit_distance &&
            last_min_distance > -1 && min_dist < last_min_distance)
    {
        std::ostringstream oss;
        oss << task_to_ignore;
        tsc->deactivateTask(oss.str());
    }

    if (predicted_distance > min_dist && min_dist > min_dist)
    {
        tsc->activateHighestPrioTask();
    }

    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(this->jacobian_data_.cols(), 1);

    if(min_dist >= crit_distance || predicted_distance > crit_distance)
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

        double magnitude = pow((2.0 * crit_distance / min_dist), 2.0) - 1.0;
        ROS_INFO_STREAM(">>>>>>>>>>>>>>>> Constraint with magnitude: " << magnitude);
        qdots_out.col(0) = q_i + exp(params_.mu) * last_jac * last_in_cart_velocities + magnitude * k_H * projector_i * q_dot_0;
        // qdots_out.col(0) = q_i + exp(params_.mu) * jacobianPseudoInverse * (last_in_cart_velocities + params_.conv_speed * params_.delta_p_vec) + magnitude * k_H * projector_i * q_dot_0;
    }
//    else if(min_dist >= 0.005)
//    {
//        double magnitude = 1.0;
//        // Deactivation of Main task
//        ROS_INFO_STREAM(">>>>>>>>>>>>>>>> Deactivation of main task!!!" << std::endl << "Internal motion with magnitude: " << magnitude);
//        qdots_out = magnitude * k_H * projector * q_dot_0;
//    }
    else
    {
        ROS_ERROR_STREAM(">>>>>>>>>>>>>>>> Deactivation of task AND constrain -> STOP ALL MOTIONS !!!");
    }

    last_in_cart_velocities = in_cart_velocities;

    last_jac = jacobianPseudoInverse;

    last_min_distance = min_dist;

    last_integration_time = now_integration_time;

    return qdots_out;
}


