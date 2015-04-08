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
 *   Implementation of the basic functionality of the solver factory
 *
 ****************************************************************/
#include "ros/ros.h"
#include "cob_twist_controller/constraint_solvers/factories/solver_factory.h"

/**
 * The base calculation method to calculate joint velocities out of input velocities (cartesian space).
 * Creates a solver according to implemented createSolver-method (in subclass).
 * Use the specialized solve-method to calculate new joint velocities.
 */
Eigen::MatrixXd SolverFactory::calculateJointVelocities(AugmentedSolverParams &asSolverParams,
                                                        Matrix6Xd &jacobianData,
                                                        Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed,
                                                        const Eigen::VectorXd &inCartVelocities,
                                                        const KDL::JntArray& q,
                                                        const KDL::JntArray& last_q_dot,
                                                        double dampingFactor)
{
    ConstraintSolver* cs = this->createSolver(asSolverParams, jacobianData, jacobianDataTransposed);
    cs->setDampingFactor(dampingFactor);
    Eigen::MatrixXd new_q_dot = cs->solve(inCartVelocities, q, last_q_dot);
    delete cs;
    cs = NULL;
    return new_q_dot;
}

SolverFactory::~SolverFactory()
{

}
