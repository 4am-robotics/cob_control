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

#include <Eigen/Dense>

/**
 * Solve the inverse differential kinematics equation by using a stack of tasks and additional constraints
 */

// Task split up into POSITION and ORIENTATION
//Eigen::MatrixXd StackOfTasksSolver::solve(const t_Vector6d &in_cart_velocities,
//                                          const KDL::JntArray& q,
//                                          const KDL::JntArray& last_q_dot) const
//{
//    static t_Vector6d last_in_cart_velocities = t_Vector6d::Zero();
//    ROS_INFO_STREAM("last_in_cart_velocities: " << last_in_cart_velocities.transpose());
//    double k_H;
//    double V_q;
//
//    double crit_distance;
//    double min_dist;
//
//    t_Vector6d tmp_in_cart_velocities = in_cart_velocities;
//    Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(q.rows());
//    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
//    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
//    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
//    Eigen::MatrixXd partialSolution = jacobianPseudoInverse * in_cart_velocities;
//    Eigen::MatrixXd homogeneousSolution = Eigen::MatrixXd::Zero(partialSolution.rows(), partialSolution.cols());
//
//    ROS_INFO_STREAM("partialSolution: " << partialSolution.transpose());
//
//    for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
//    {
//        q_dot_0 = (*it)->getPartialValues();
//        crit_distance = (*it)->getActivationThreshold();
//        min_dist = (*it)->getValue();
//        Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
//        k_H = (*it)->getSelfMotionMagnitude(partialSolution, tmpHomogeneousSolution);
//        homogeneousSolution += (k_H * tmpHomogeneousSolution);
//
//        V_q = (*it)->getValue();
//    }
//
//    Eigen::VectorXd q_i = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
//    Eigen::MatrixXd projector_i = Eigen::MatrixXd::Identity(this->jacobian_data_.cols(), this->jacobian_data_.cols());
//
//    int32_t task_to_ignore = -1;
//    double crit = 9999999;
//    std::vector<int32_t> tasks_to_ignore;
//    for(int32_t taskNr = 0; taskNr < 2; ++taskNr) // TODO: where to get max number of tasks? A POSITION AND A ORIENTATION TASK
//    {
//        Eigen::MatrixXd J_task = this->jacobian_data_.block(taskNr * 3, 0, 3, this->jacobian_data_.cols());
//        ROS_INFO_STREAM("Task Jacobian: " << std::endl << J_task);
//
//        Eigen::MatrixXd J_task_inv = pinv_calc_.calculate(this->params_, this->damping_, J_task);
//        Eigen::MatrixXd projector_task = Eigen::MatrixXd::Identity(J_task_inv.rows(), J_task.cols()) - J_task_inv * J_task;
//        Eigen::VectorXd pg = projector_task * q_dot_0;
//
//        Eigen::VectorXd left = J_task_inv * in_cart_velocities.block(taskNr * 3, 0, 3, 1);
//        double scalar = left.dot(q_dot_0); // opposite direction?
//
//        if (q_dot_0.norm() > 0.0 && crit > pg.norm() && scalar < 0.0)
//        {
//            crit = pg.norm();
//            task_to_ignore = taskNr;
//            tasks_to_ignore.push_back(taskNr);
//        }
//    }
//
//
//    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(this->jacobian_data_.cols(), 1);
//
//    if(min_dist >= 0.015)
//    {
//
//        std::vector<int32_t>::iterator it;
//        for(int32_t taskNr = 0; taskNr < 2; ++taskNr) // TODO: where to get max number of tasks?
//        {
//            if (task_to_ignore == taskNr)
//            {
//                ROS_WARN_STREAM("Ignoring task number: " << taskNr);
//                continue;
//            }
//
//    //        if(std::find(tasks_to_ignore.begin(), tasks_to_ignore.end(), taskNr) != tasks_to_ignore.end())
//    //        {
//    //            ROS_WARN_STREAM("Ignoring task number: " << taskNr);
//    //            continue;
//    //        }
//
//
//            Eigen::MatrixXd J_task = this->jacobian_data_.block(taskNr * 3, 0, 3, this->jacobian_data_.cols());
//            Eigen::MatrixXd J_temp = J_task * projector_i;
//            Eigen::MatrixXd J_temp_inv = pinv_calc_.calculate(this->params_, this->damping_, J_temp);
//
//
//
//            Eigen::MatrixXd v_task = in_cart_velocities.block(taskNr * 3, 0, 3, 1);
//
//            q_i = q_i + J_temp_inv * (v_task - J_task * q_i);
//            projector_i = projector_i - J_temp_inv * J_temp;
//        }
//
//        double magnitude = (pow(crit_distance / min_dist, 2.0) - 1.0);
//        //qdots_out.col(0) = q_i + exp(-100.0 * 0.02) * jacobianPseudoInverse * in_cart_velocities + magnitude * k_H * projector_i * q_dot_0;
//        ROS_INFO_STREAM(">>>>>>>>>>>>>>>> Constraint with magnitude: " << magnitude);
//        qdots_out.col(0) = q_i + magnitude * k_H * projector_i * q_dot_0;
//        // exp(-300.0 * 0.02) * jacobianPseudoInverse * last_in_cart_velocities +
//    }
//    else if(min_dist >= 0.005)
//    {
//        double magnitude = 1.0;
//        // Deactivation of Main task
//        ROS_INFO_STREAM(">>>>>>>>>>>>>>>> Deactivation of main task!!!" << std::endl << "Internal motion with magnitude: " << magnitude);
//        qdots_out = magnitude * k_H * projector * q_dot_0;
//    }
//    else
//    {
//        ROS_ERROR_STREAM(">>>>>>>>>>>>>>>> Deactivation of task AND constrain -> STOP ALL MOTIONS !!!");
//    }
//
//    ROS_INFO_STREAM("qdots_out: " << qdots_out.transpose());
//    //qdots_out.col(0) = q_i;
//
//    last_in_cart_velocities = in_cart_velocities;
//
//    return qdots_out;
//}



/*
 * Split all cart velocities into separate tasks
 *
 *
 *
 *
 *
 *
 */
Eigen::MatrixXd StackOfTasksSolver::solve(const t_Vector6d &in_cart_velocities,
                                          const KDL::JntArray& q,
                                          const KDL::JntArray& last_q_dot) const
{
    static t_Vector6d last_in_cart_velocities = t_Vector6d::Zero();
    ROS_INFO_STREAM("last_in_cart_velocities: " << last_in_cart_velocities.transpose());
    double k_H;
    double V_q;

    double crit_distance;
    double min_dist;

    t_Vector6d tmp_in_cart_velocities = in_cart_velocities;
    Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(q.rows());
    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
    Eigen::MatrixXd partialSolution = jacobianPseudoInverse * in_cart_velocities;
    Eigen::MatrixXd homogeneousSolution = Eigen::MatrixXd::Zero(partialSolution.rows(), partialSolution.cols());


    Eigen::ColPivHouseholderQR<Eigen::Matrix<double,7,7> > qr1(projector);
    qr1.setThreshold(1e-5);
    ROS_INFO_STREAM("Rank of the Null-Space Projector: " <<  int(qr1.rank()));


    ROS_INFO_STREAM("partialSolution: " << partialSolution.transpose());

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
    std::vector<int32_t> tasks_to_ignore;
    for(int32_t taskNr = 0; taskNr < in_cart_velocities.rows(); ++taskNr) // TODO: where to get max number of tasks?
    {
        Eigen::MatrixXd J_task = this->jacobian_data_.row(taskNr);
        Eigen::MatrixXd J_task_inv = pinv_calc_.calculate(this->params_, this->damping_, J_task);
        Eigen::MatrixXd projector_task = Eigen::MatrixXd::Identity(J_task_inv.rows(), J_task.cols()) - J_task_inv * J_task;

        Eigen::VectorXd pg = projector_task * q_dot_0;

        Eigen::VectorXd left = J_task_inv * in_cart_velocities.row(taskNr);
        double scalar = left.dot(q_dot_0); // opposite direction?

        if (q_dot_0.norm() > 0.0 && crit > pg.norm() && scalar < 0.0)
        {
            crit = pg.norm();
            task_to_ignore = taskNr;
            tasks_to_ignore.push_back(taskNr);
        }

    }

//    Eigen::MatrixXd J_task0 = q_dot_0.transpose();
//    Eigen::MatrixXd J_task0_inv = pinv_calc_.calculate(this->params_, this->damping_, J_task0);
//    q_i = q_i + J_task0_inv * (k_H * V_q);
//    projector_i = projector_i - J_task0_inv * J_task0;
    //ROS_INFO_STREAM("projector_i: " << projector_i);

    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(this->jacobian_data_.cols(), 1);

    if(min_dist >= 0.015)
    {

        std::vector<int32_t>::iterator it;
        for(int32_t taskNr = 0; taskNr < in_cart_velocities.rows(); ++taskNr) // TODO: where to get max number of tasks?
        {
            if (task_to_ignore == taskNr)
            {
                ROS_WARN_STREAM("Ignoring task number: " << taskNr);
                continue;
            }

    //        if(std::find(tasks_to_ignore.begin(), tasks_to_ignore.end(), taskNr) != tasks_to_ignore.end())
    //        {
    //            ROS_WARN_STREAM("Ignoring task number: " << taskNr);
    //            continue;
    //        }


            Eigen::MatrixXd J_task = this->jacobian_data_.row(taskNr);
            Eigen::MatrixXd J_temp = J_task * projector_i;
            Eigen::MatrixXd J_temp_inv = pinv_calc_.calculate(this->params_, this->damping_, J_temp);
            Eigen::MatrixXd v_task = in_cart_velocities.row(taskNr);

            q_i = q_i + J_temp_inv * (v_task - J_task * q_i);
            projector_i = projector_i - J_temp_inv * J_temp;
        }

        double magnitude = (pow(crit_distance / min_dist, 2.0));
        //qdots_out.col(0) = q_i + exp(-100.0 * 0.02) * jacobianPseudoInverse * in_cart_velocities + magnitude * k_H * projector_i * q_dot_0;
        ROS_INFO_STREAM(">>>>>>>>>>>>>>>> Constraint with magnitude: " << magnitude);

        ROS_INFO_STREAM(">>>>>>> Last in cart velocities: " << std::endl << last_in_cart_velocities);
        ROS_INFO_STREAM(">>>>>>> Last in cart pos delta: " << std::endl << params_.delta_p_vec);

        qdots_out.col(0) = q_i + exp(params_.mu) * jacobianPseudoInverse * (last_in_cart_velocities + params_.conv_speed * params_.delta_p_vec) + magnitude * k_H * projector_i * q_dot_0;
        //

        Eigen::ColPivHouseholderQR<Eigen::Matrix<double,7,7> > qr(projector_i);
        qr.setThreshold(1e-5);
        ROS_INFO_STREAM("Rank of the recursive Null-Space Projector: " <<  int(qr.rank()));

    }
    else if(min_dist >= 0.005)
    {
        double magnitude = 1.0;
        // Deactivation of Main task
        ROS_INFO_STREAM(">>>>>>>>>>>>>>>> Deactivation of main task!!!" << std::endl << "Internal motion with magnitude: " << magnitude);
        qdots_out = magnitude * k_H * projector * q_dot_0;
    }
    else
    {
        ROS_ERROR_STREAM(">>>>>>>>>>>>>>>> Deactivation of task AND constrain -> STOP ALL MOTIONS !!!");
    }

    ROS_INFO_STREAM("qdots_out: " << qdots_out.transpose());
    //qdots_out.col(0) = q_i;

    last_in_cart_velocities = in_cart_velocities;

    return qdots_out;
}


//Eigen::MatrixXd StackOfTasksSolver::solve(const t_Vector6d &in_cart_velocities,
//                                          const KDL::JntArray& q,
//                                          const KDL::JntArray& last_q_dot) const
//{
//    double k_H;
//    double V_q;
//    Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(q.rows());
//    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
//    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
//    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
//    Eigen::MatrixXd partialSolution = jacobianPseudoInverse * in_cart_velocities;
//    Eigen::MatrixXd homogeneousSolution = Eigen::MatrixXd::Zero(partialSolution.rows(), partialSolution.cols());
//
//    ROS_INFO_STREAM("partialSolution: " << partialSolution.transpose());
//
//    for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
//    {
//        q_dot_0 = (*it)->getPartialValues();
//        Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
//        k_H = (*it)->getSelfMotionMagnitude(partialSolution, tmpHomogeneousSolution);
//        homogeneousSolution += (k_H * tmpHomogeneousSolution);
//
//        V_q = (*it)->getValue();
//    }
//
//    ROS_INFO_STREAM("q_dot_0: " << std::endl << q_dot_0);
//    ROS_INFO_STREAM("in_cart_velocities: " << std::endl << in_cart_velocities);
//    ROS_INFO_STREAM("this->jacobian_data_: " << std::endl << this->jacobian_data_);
//
//    Eigen::VectorXd q_i = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
//    Eigen::MatrixXd projector_i = Eigen::MatrixXd::Identity(this->jacobian_data_.cols(), this->jacobian_data_.cols());
//
//    int32_t task_to_ignore = -1;
//    double crit = 9999999;
//    for(int32_t taskNr = 0; taskNr < in_cart_velocities.rows(); ++taskNr) // TODO: where to get max number of tasks?
//    {
//        Eigen::MatrixXd J_task = this->jacobian_data_.row(taskNr);
//        Eigen::MatrixXd J_task_inv = pinv_calc_.calculate(this->params_, this->damping_, J_task);
//        Eigen::MatrixXd projector_task = Eigen::MatrixXd::Identity(J_task_inv.rows(), J_task.cols()) - J_task_inv * J_task;
//
//        Eigen::VectorXd pg = projector_task * q_dot_0;
//        if (q_dot_0.norm() > 0.0 && crit > pg.norm())
//        {
//            ROS_WARN_STREAM("Found a task to ignore: " << taskNr);
//            crit = pg.norm();
//            task_to_ignore = taskNr;
//        }
//    }
//
//
//    Eigen::MatrixXd J_task0 = q_dot_0.transpose();
//    Eigen::MatrixXd J_task0_inv = pinv_calc_.calculate(this->params_, this->damping_, J_task0);
//    q_i = q_i + J_task0_inv * (k_H * V_q);
//    projector_i = projector_i - J_task0_inv * J_task0;
//
//    Eigen::MatrixXd J_sum = Eigen::MatrixXd::Zero(this->jacobian_data_.rows() + 1, this->jacobian_data_.cols());
//    J_sum.row(0) << J_task0;
//    ROS_INFO_STREAM("projector_i: " << projector_i);
//
//    J_sum.block(1, 0, this->jacobian_data_.rows(), this->jacobian_data_.cols()) << this->jacobian_data_;
//
//    ROS_INFO_STREAM("J_sum: " << J_sum);
//
//    for(int32_t taskNr = 0; taskNr < in_cart_velocities.rows(); ++taskNr) // TODO: where to get max number of tasks?
//    {
//        if (task_to_ignore == taskNr)
//        {
//            ROS_WARN_STREAM("Ignoring task number: " << taskNr);
//            continue;
//        }
//
//        Eigen::MatrixXd J_task = this->jacobian_data_.row(taskNr);
//        Eigen::MatrixXd J_temp = J_task * projector_i;
//        Eigen::MatrixXd J_temp_inv = pinv_calc_.calculate(this->params_, this->damping_, J_temp);
//        Eigen::MatrixXd v_task = in_cart_velocities.row(taskNr);
//
//        q_i = q_i + J_temp_inv * (v_task - J_task * q_i);
//        projector_i = projector_i - J_temp_inv * J_temp;
//    }
//
//    Eigen::MatrixXd task_vars = Eigen::MatrixXd::Zero(in_cart_velocities.rows() + 1, 1);   // << k_H * V_q, in_cart_velocities;
//    task_vars(0,0) = k_H * V_q;
//    for (int i = 1; i < task_vars.rows(); ++i)
//    {
//        task_vars(i) = in_cart_velocities(i-1);
//    }
//
//    ROS_INFO_STREAM("task_vars: " << task_vars);
//
//    Eigen::MatrixXd G_A = pinv_calc_.calculate(this->params_, this->damping_, J_sum);
//    q_i = q_i + G_A * exp(-14.0) * task_vars;
//
//    ROS_INFO_STREAM("q_n: " << q_i.transpose());
//
//    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(q_i.rows(), 1);
//    //qdots_out.col(0) = q_i + homogeneousSolution;
//    qdots_out.col(0) = q_i;
//
//    // Eigen::MatrixXd qdots_out = partialSolution + homogeneousSolution; // weighting with k_H is done in loop
//    return qdots_out;
//}
