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
 *   This header contains the interface description of constraint solvers
 *   Pure virtual methods have to be implemented in subclasses
 *
 ****************************************************************/
#ifndef CONSTRAINT_SOLVER_BASE_H_
#define CONSTRAINT_SOLVER_BASE_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/jntarray.hpp>
#include "cob_twist_controller/augmented_solver_data_types.h"

class ConstraintSolver
{
    public:
        virtual Eigen::MatrixXd solve(const Eigen::VectorXd &inCartVelocities, const KDL::JntArray& q, const KDL::JntArray& q_dot) const = 0; // TODO: Check VectorXd for q_dot?
        inline void setDampingFactor(double damping)
        {
            this->dampingFactor_ = damping;
        }

        static const double DAMPING_LIMIT = 1.0e-9;

    protected:

        ConstraintSolver(AugmentedSolverParams &asSolverParams,
                         Matrix6Xd &jacobianData,
                         Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed);
        virtual ~ConstraintSolver() = 0;

        Eigen::MatrixXd calculatePinvJacobianBySVD(Eigen::JacobiSVD<Eigen::MatrixXd> svd) const;

        const AugmentedSolverParams &asSolverParams_;
        const Matrix6Xd &jacobianData_;
        const Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed_;
        double dampingFactor_;
};

#endif /* CONSTRAINT_SOLVER_BASE_H_ */
