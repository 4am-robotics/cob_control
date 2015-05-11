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
    for (std::set<tConstraintBase>::const_iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        // ROS_INFO_STREAM("" << lv++ << ") GradientProjectionMethodSolver::solve: " << (*it)->getValue() << std::endl);
    }

    return Eigen::MatrixXd::Zero(7,1);
}
