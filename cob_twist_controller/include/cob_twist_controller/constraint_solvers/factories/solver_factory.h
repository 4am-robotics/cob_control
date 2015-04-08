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
 *   This header contains the interface description to create solvers
 *
 ****************************************************************/
#ifndef SOLVER_FACTORY_H_
#define SOLVER_FACTORY_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/jntarray.hpp>
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

/// Abstract base class defining interfaces for the creation of a specific solver.
class SolverFactory
{
    public:

        /**
         * @param asSolverParams References the augmented solver parameters.
         * @param jacobianData References the current Jacobian (matrix data only).
         * @param jacobianDataTransposed References the current Jacobian transpose (matrix data only).
         * @param inCartVelocities The input velocities vector (in cartesian space).
         * @param q The current joint positions.
         * @param last_q_dot The last joint velocities.
         * @param dampingFactor The damping factor corresponding to damping method.
         * @return Joint velocities in a (m x 1)-Matrix.
         */
        Eigen::MatrixXd calculateJointVelocities(AugmentedSolverParams &asSolverParams,
                                                 Matrix6Xd &jacobianData,
                                                 Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed,
                                                 const Eigen::VectorXd &inCartVelocities,
                                                 const KDL::JntArray& q,
                                                 const KDL::JntArray& last_q_dot,
                                                 double dampingFactor);

        virtual ~SolverFactory() = 0;

    protected:

        /**
         * The interface method to create a specific solver to solve the inverse kinematics problem.
         * @param asSolverParams References the augmented solver parameters.
         * @param jacobianData References the current Jacobian (matrix data only).
         * @param jacobianDataTransposed References the current Jacobian transpose (matrix data only).
         * @return A specific solver.
         */
        virtual ConstraintSolver* createSolver(AugmentedSolverParams &asSolverParams,
                                               Matrix6Xd &jacobianData,
                                               Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed) const = 0;

};

#endif /* SOLVER_FACTORY_H_ */
