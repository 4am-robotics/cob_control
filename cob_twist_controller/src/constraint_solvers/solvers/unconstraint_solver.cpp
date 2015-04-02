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
 *   Implementation of an unconstraint solver.
 *
 ****************************************************************/
#include "cob_twist_controller/constraint_solvers/solvers/unconstraint_solver.h"

UnconstraintSolver::UnconstraintSolver(AugmentedSolverParams &asSolverParams,
                                       Matrix6Xd &jacobianData,
                                       Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed)
    : ConstraintSolver(asSolverParams,
                       jacobianData,
                       jacobianDataTransposed)
{

}


UnconstraintSolver::~UnconstraintSolver()
{

}


Eigen::MatrixXd UnconstraintSolver::solve(const Eigen::VectorXd& inCartVelocities, const KDL::JntArray& q, const KDL::JntArray& q_dot) const
{
    double eps = this->asSolverParams_.eps; // Truncation always active!!!
    uint32_t cols = this->jacobianData_.cols(); // in original implementation KDL::jacobian.columns has been taken!
    uint32_t rows = this->jacobianData_.rows(); // in original implementation KDL::jacobian.rows has been taken!
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->jacobianData_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd jacobianPseudoInverse = this->calculatePinvJacobianBySVD(svd);
    Eigen::MatrixXd qdots_out = jacobianPseudoInverse * inCartVelocities;
    return qdots_out;
}

