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
 *   This header contains the description of the unconstraint solver
 *   Implements methods from constraint_solver_base
 *
 ****************************************************************/
#include "cob_twist_controller/constraint_solvers/solvers/gradient_projection_method_solver.h"

Eigen::MatrixXd GradientProjectionMethodSolver::solve(const Eigen::VectorXd &inCartVelocities,
                                      const KDL::JntArray& q,
                                      const KDL::JntArray& last_q_dot) const
{
    double kappa;
    Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(q.rows());
    Eigen::MatrixXd jacobianPseudoInverse = pinvCalc_.calculate(this->asParams_, this->damping_, this->jacobianData_);
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobianData_.cols());
    Eigen::MatrixXd projector = ident - jacobianPseudoInverse * this->jacobianData_;
    Eigen::MatrixXd partialSolution = jacobianPseudoInverse * inCartVelocities;
    Eigen::MatrixXd homogeneousSolution = Eigen::MatrixXd::Zero(partialSolution.rows(), partialSolution.cols());

    for (std::set<tConstraintBase>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        q_dot_0 = (*it)->getPartialValues();
        Eigen::MatrixXd tmpHomogeneousSolution = projector * q_dot_0;
        homogeneousSolution += tmpHomogeneousSolution;
        kappa = (*it)->getSelfMotionMagnitude(partialSolution, tmpHomogeneousSolution);
    }

    if(q_dot_0.norm() > 0.00001)
    //if(false)
    {
        ROS_INFO_STREAM("jacobianPseudoInverse: " << std::endl << jacobianPseudoInverse);
        ROS_INFO_STREAM("this->jacobianData_: " << std::endl << this->jacobianData_);
        ROS_INFO_STREAM("projector: " << std::endl << projector);
        ROS_INFO_STREAM("q_dot_0: " << std::endl << q_dot_0);
        ROS_INFO_STREAM("kappa: " << std::endl << kappa);
        ROS_INFO_STREAM("partialSolution: " << std::endl << partialSolution);
        ROS_INFO_STREAM("homogeneousSolution: " << std::endl << homogeneousSolution);
    }

    Eigen::MatrixXd qdots_out = partialSolution + kappa * homogeneousSolution;
    return qdots_out;
}
