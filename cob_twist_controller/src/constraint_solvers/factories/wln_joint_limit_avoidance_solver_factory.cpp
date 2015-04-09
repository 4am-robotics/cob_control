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
 *   Implementation of a factory to solve JLA problems.
 *
 ****************************************************************/
#include "cob_twist_controller/constraint_solvers/factories/wln_joint_limit_avoidance_solver_factory.h"
#include "cob_twist_controller/constraint_solvers/solvers/wln_joint_limit_avoidance_solver.h"

ConstraintSolver* WLN_JointLimitAvoidanceSolverFactory::createSolver(AugmentedSolverParams &asSolverParams,
                                               Matrix6Xd &jacobianData,
                                               Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed) const
{
    return new WLN_JointLimitAvoidanceSolver(asSolverParams, jacobianData, jacobianDataTransposed);
}
