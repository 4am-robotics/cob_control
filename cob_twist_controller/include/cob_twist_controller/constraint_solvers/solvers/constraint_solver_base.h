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
#include <kdl/jntarray.hpp>
#include <boost/shared_ptr.hpp>
#include <cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h>
#include "cob_twist_controller/damping_methods/damping_base.h"
#include "cob_twist_controller/augmented_solver_data_types.h"

/// Base class for solvers, defining interface methods.
template <typename PINV = PInvBySVD>
class ConstraintSolver
{
    public:
        /**
         * The interface method to solve the inverse kinematics problem. Has to be implemented in inherited classes.
         * @param inCartVelocities The input velocities vector (in cartesian space).
         * @param q The current joint positions.
         * @param last_q_dot The last joint velocities.
         * @return The calculated new joint velocities.
         */
        virtual Eigen::MatrixXd solve(const Eigen::VectorXd &inCartVelocities,
                                      const KDL::JntArray& q,
                                      const KDL::JntArray& last_q_dot,
                                      const Eigen::VectorXd &tracking_errors) const = 0;

        /**
         * Inline method to set the damping
         * @param damping The new damping
         */
        inline void setDamping(boost::shared_ptr<DampingBase>& damping)
        {
            this->damping_ = damping;
        }

        virtual ~ConstraintSolver() {}

    protected:

        ConstraintSolver(AugmentedSolverParams &asParams,
                         Matrix6Xd &jacobianData)
                         : asParams_(asParams),
                           jacobianData_(jacobianData)
        {
        }

        /**
         * Base method for calculation of the pseudoinverse Jacobian by using SVD.
         * @param jacobian The Jacobi matrix.
         * @return A pseudoinverse Jacobian
         */
        inline Eigen::MatrixXd calculatePinvJacobianBySVD(const Eigen::MatrixXd& jacobian) const
        {
            return pinvCalc_.calculate(this->asParams_, this->damping_, jacobian);
        }

        const AugmentedSolverParams& asParams_; ///< References the augmented solver parameters.
        const Matrix6Xd& jacobianData_; ///< References the current Jacobian (matrix data only).
        boost::shared_ptr<DampingBase> damping_; ///< The currently set damping method.

        PINV pinvCalc_;
};

#endif /* CONSTRAINT_SOLVER_BASE_H_ */
