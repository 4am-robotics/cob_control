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
#include "cob_twist_controller/constraint_solvers/solvers/xxx_task_priority_solver.h"

/**
 * Solve the inverse differential kinematics equation by using a two tasks.
 * Maciejewski A., Obstacle Avoidance for Kinematically Redundant Manipulators in Dyn Varying Environments.
 */
//
Eigen::MatrixXd TaskPrioritySolver::solve(const t_Vector6d &in_cart_velocities,
                                          const KDL::JntArray& q,
                                          const KDL::JntArray& last_q_dot) const
{

    Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(q.rows(), 1);

    double k_H;
    double crit_distance;
    Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(q.rows());
    Eigen::Vector3d distance;
    double min_dist;

    Eigen::MatrixXd obst_dist_pnt_jac;
    Eigen::MatrixXd jacobianPseudoInverse = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobian_data_;
    Eigen::MatrixXd partialSolution = jacobianPseudoInverse * in_cart_velocities;

    if (this->constraints_.size() > 0)
    {
        for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
        {
            //ROS_INFO_STREAM("Found constraint");
            q_dot_0 = (*it)->getPartialValues();
            distance = (*it)->getDistanceVector();
            obst_dist_pnt_jac = (*it)->getObstacleAvoidancePointJac();
            crit_distance = (*it)->getActivationThreshold();
            min_dist = (*it)->getValue();

        }

        t_Vector6d ext_distance;

        //ROS_INFO_STREAM("q_dot_0: " << q_dot_0);
        //ROS_INFO_STREAM("distance: " << distance);

        for (int i = 0; i < distance.rows(); ++i)
        {
            ext_distance(i, 0) = distance(i);
            ext_distance(i+3, 0) = 0.0;
        }

        //ROS_INFO_STREAM("Ext distance: " << ext_distance);

        double norm_ext_dist = ext_distance.norm();
        double magnitude = 0.0;
        double activation = 0.0;

        //ROS_INFO_STREAM("norm_ext_dist: " << norm_ext_dist);
        //ROS_INFO_STREAM("crit_distance: " << crit_distance);
        if(min_dist <= crit_distance)
        //if(false)
        {


            Eigen::Matrix3Xd m = Eigen::Matrix3Xd::Zero(3, 7); // TODO: Not const 7 bad style
            m << obst_dist_pnt_jac.row(0),
                 obst_dist_pnt_jac.row(1),
                 obst_dist_pnt_jac.row(2);

            //Eigen::MatrixXd tmp_matrix = m * projector;
            Eigen::MatrixXd tmp_matrix = obst_dist_pnt_jac * projector;

            //ROS_INFO_STREAM("tmp_matrix: " << tmp_matrix);
            Eigen::MatrixXd jac_inv_2nd_term = pinv_calc_.calculate(this->params_, this->damping_, tmp_matrix);

            //ROS_INFO_STREAM("jac_inv_2nd_term: " << jac_inv_2nd_term);




            if (min_dist > 0.015)
            {
                t_Vector6d unit_direction = ext_distance / norm_ext_dist;
                // Eigen::Vector3d unit_direction = distance / norm_ext_dist;

                //ROS_INFO_STREAM("unit_direction: " << unit_direction.transpose());

                magnitude = pow(crit_distance / min_dist, 2.0) - 1.0;
                activation = 1.0;

                //ROS_INFO_STREAM("magnitude: " << std::endl << magnitude);
                //ROS_INFO_STREAM("norm_ext_dist: " << std::endl << norm_ext_dist);
                //ROS_INFO_STREAM("jacobianPseudoInverse: " << std::endl << jacobianPseudoInverse);
                //ROS_INFO_STREAM("this->jacobianData_: " << std::endl << this->jacobian_data_);
                //ROS_INFO_STREAM("projector: " << std::endl << projector);
                // ROS_INFO_STREAM("Last k_H: " << std::endl << k_H);
                //ROS_INFO_STREAM("partialSolution: " << std::endl << partialSolution);
                //ROS_INFO_STREAM("obst_dist_pnt_jac: " << std::endl << obst_dist_pnt_jac);
                //ROS_INFO_STREAM("tmp_matrix: " << std::endl << tmp_matrix);
                //ROS_INFO_STREAM("jac_inv_2nd_term: " << std::endl << jac_inv_2nd_term);

                //Eigen::MatrixXd tmp2 = m * partialSolution;
                qdots_out = partialSolution + activation * jac_inv_2nd_term * (magnitude * unit_direction - obst_dist_pnt_jac * partialSolution);



                ROS_INFO_STREAM("[min_dist > 0.015] qdots_out: " << std::endl << qdots_out);
            }
            else
            {
                t_Vector6d unit_direction = ext_distance;
                qdots_out = jac_inv_2nd_term * (10.0 * unit_direction - obst_dist_pnt_jac * partialSolution);


                ROS_WARN_STREAM("[min_dist < 0.015] qdots_out: " << std::endl << qdots_out);
            }


        }
        else
        {
            qdots_out = partialSolution;
            ROS_INFO_STREAM("Normal Solution: " << std::endl << qdots_out);
        }
    }
    else
    {

        qdots_out = partialSolution;
        ROS_ERROR_STREAM("Should not occur solution: " << std::endl << qdots_out);
    }





//    Eigen::MatrixXd qdots_out = partialSolution + homogeneousSolution; // weighting with k_H is done in loop
    return qdots_out;
}

