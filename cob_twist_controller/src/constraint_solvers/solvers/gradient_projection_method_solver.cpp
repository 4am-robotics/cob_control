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
                                      const KDL::JntArray& last_q_dot,
                                      const Eigen::VectorXd &tracking_errors) const
{
    uint16_t lv = 1;
    Eigen::VectorXd q_dot_0 = Eigen::VectorXd::Zero(q.rows());
    for (std::set<tConstraintBase>::const_iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        double stepSize = (*it)->getStepSize();
        q_dot_0 += -stepSize * (*it)->getPartialValues();
        //ROS_INFO_STREAM("" << lv++ << ") GradientProjectionMethodSolver::solve: " << std::endl << (*it)->getPartialValues() << std::endl);
    }

    if(q_dot_0.norm() > 0.00001)
    {
        ROS_INFO_STREAM("q_dot_0: " << std::endl << q_dot_0);
    }

    Eigen::MatrixXd jacobianPseudoInverse = pinvCalc_.calculate(this->asParams_, this->damping_, this->jacobianData_);
    // Eigen::MatrixXd qdots_out = jacobianPseudoInverse * (inCartVelocities - this->asParams_.p_gain * tracking_errors);

    //ROS_INFO_STREAM("Got q_dot_0: " << std::endl << q_dot_0 << std::endl);


    Eigen::MatrixXd tmp = Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), this->jacobianData_.cols()) - jacobianPseudoInverse * this->jacobianData_;

    //ROS_INFO_STREAM("Gradient: " << std::endl << q_dot_0 << std::endl);
//    q_dot_0(0) = 1.0;
//    q_dot_0(1) = 0.0;
//    q_dot_0(2) = -1.0;
//    q_dot_0(3) = 0.0;
//    q_dot_0(4) = 1.0;
//    q_dot_0(5) = 0.0;
//    q_dot_0(6) = -1.0;
    Eigen::MatrixXd tmp_q_dot = tmp * q_dot_0;

    ROS_INFO_STREAM("2nd sum term: " << std::endl << tmp_q_dot << std::endl);

    // Eigen::MatrixXd qdots_out = Eigen::MatrixXd::Zero(tmp_q_dot.rows(), tmp_q_dot.cols());
    Eigen::MatrixXd qdots_out = jacobianPseudoInverse * inCartVelocities;
//    if (double(q_dot_0.norm()) < 0.00001 )
//    {
//        qdots_out =
//    }
//    else
//    {
//        ROS_INFO_STREAM("Found a gradient setting ee vel to 0 allow null-space motion!");
//        // qdots_out = zeroVec;
//    }

    qdots_out += tmp_q_dot;

    // (Eigen::MatrixXd::Identity(jacobianPseudoInverse.rows(), jacobianPseudoInverse.cols()) + jacobianPseudoInverse * this->jacobianData_) * q_dot_0;

    return qdots_out;
}
