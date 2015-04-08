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
 *   This header contains the description of the JLA solver
 *   Implements methods from constraint_solver_base
 *   Special constraint handling.
 *
 ****************************************************************/
#ifndef JOINT_LIMIT_AVOIDANCE_SOLVER_H_
#define JOINT_LIMIT_AVOIDANCE_SOLVER_H_

#include "cob_twist_controller/augmented_solver_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

/// Implementation of ConstraintSolver to solve inverse kinematics with joint limit avoidance
class JointLimitAvoidanceSolver : public ConstraintSolver
{
    public:

        /**
         * Specific implementation of solve-method to solve IK problem with joint limit avoidance.
         * See base class ConstraintSolver for more details on params and returns.
         */
        virtual Eigen::MatrixXd solve(const Eigen::VectorXd &inCartVelocities, const KDL::JntArray& q, const KDL::JntArray& last_q_dot) const;

        JointLimitAvoidanceSolver(AugmentedSolverParams &asSolverParams,
                                  Matrix6Xd &jacobianData,
                                  Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed)
                                  : ConstraintSolver(asSolverParams,
                                                     jacobianData,
                                                     jacobianDataTransposed)
        {

        }

        virtual ~JointLimitAvoidanceSolver()
        {

        }


    private:

        /**
         * Helper method that calculates a weighting for the Jacobian to adapt the impact on joint velocities.
         * @param q The current joint positions.
         * @param q_dot The current joint velocities.
         * @return Diagonal weighting matrix that adapts the Jacobian.
         */
        Eigen::MatrixXd calculateWeighting(const KDL::JntArray& q, const KDL::JntArray& q_dot) const;
};

#endif /* JOINT_LIMIT_AVOIDANCE_SOLVER_H_ */
