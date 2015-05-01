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

/// Base class for solvers, defining interface methods.
class ConstraintSolver
{
    public:
        static const double DAMPING_LIMIT = 1.0e-9; ///< const. value for zero comparison with damping factor

        /**
         * The interface method to solve the inverse kinematics problem. Has to be implemented in inherited classes.
         * @param inCartVelocities The input velocities vector (in cartesian space).
         * @param q The current joint positions.
         * @param last_q_dot The last joint velocities.
         * @return The calculated new joint velocities.
         */
        virtual Eigen::MatrixXd solve(const Eigen::VectorXd &inCartVelocities, const KDL::JntArray& q, const KDL::JntArray& last_q_dot) const = 0;

        /**
         * Inline method to set the damping factor
         * @param damping The new damping factor
         */
        inline void setDampingFactor(double damping)
        {
            this->dampingFactor_ = damping;
        }

        virtual ~ConstraintSolver() = 0;

    protected:

        ConstraintSolver(AugmentedSolverParams &asParams,
                         Matrix6Xd &jacobianData)
                         : asParams_(asParams),
                           jacobianData_(jacobianData),
                           dampingFactor_(0.0)
        {

        }

        /**
         * Base method for calculation of the pseudoinverse Jacobian by using SVD.
         * @param svd The singular value decomposition object of a Jacobian.
         * @return A pseudoinverse Jacobian
         */
        Eigen::MatrixXd calculatePinvJacobianBySVD(Eigen::JacobiSVD<Eigen::MatrixXd> svd) const;


        const AugmentedSolverParams &asParams_; ///< References the augmented solver parameters.
        const Matrix6Xd &jacobianData_; ///< References the current Jacobian (matrix data only).
        double dampingFactor_; ///< The currently set damping factor.
};

#endif /* CONSTRAINT_SOLVER_BASE_H_ */
