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
 *   Implementation of constructor for constraint_solver_base.
 *
 ****************************************************************/
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

// TODO: Make this inline later ?

ConstraintSolver::ConstraintSolver(AugmentedSolverParams &asSolverParams,
                                   Matrix6Xd &jacobianData,
                                   Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed)
    : asSolverParams_(asSolverParams),
      jacobianData_(jacobianData),
      jacobianDataTransposed_(jacobianDataTransposed),
      dampingFactor_(0.0)
{

}

ConstraintSolver::~ConstraintSolver()
{

}


Eigen::MatrixXd ConstraintSolver::calculatePinvJacobianBySVD(Eigen::JacobiSVD<Eigen::MatrixXd> svd) const
{
    double eps = this->asSolverParams_.eps; // Truncation always active!!!
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(singularValues.rows());

    // Formula from Advanced Robotics : Redundancy and Optimization : Nakamura, Yoshihiko, 1991, Addison-Wesley Pub. Co [Page 258-260]
    for(uint32_t i = 0; i < singularValues.rows(); ++i)
    {
        // small change to ref: here quadratic damping due to Control of Redundant Robot Manipulators : R.V. Patel, 2005, Springer [Page 13-14]
        if (ConstraintSolver::DAMPING_LIMIT < this->dampingFactor_)
        {
            double denominator = (singularValues(i) * singularValues(i) + this->dampingFactor_ * this->dampingFactor_);
            singularValuesInv(i) = denominator < eps ? 0.0 : singularValues(i) / denominator;
        }
        else // damping seems to be disabled due to damping factor lower than a const. limit
        {
            singularValuesInv(i) = singularValues(i) < eps ? 0.0 : 1.0 / singularValues(i);
        }
    }

    Eigen::MatrixXd pseudoInverseJacobian = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    return pseudoInverseJacobian;
}
