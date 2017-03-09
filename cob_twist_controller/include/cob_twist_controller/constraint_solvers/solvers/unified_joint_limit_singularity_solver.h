/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2017 \n
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
 *   Author: Bruno Brito, email: Bruno.Brito@ipa.fraunhofer.de
 *
 * \date Date of creation: March, 2017
 *
 * \brief
 *   This header contains the interface description of constraint solvers
 *   Pure virtual methods have to be implemented in subclasses
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_UNIFIED_JOINT_LIMIT_SINGULARITY_SOLVER_H
#define COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_UNIFIED_JOINT_LIMIT_SINGULARITY_SOLVER_H

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

/// Implementation of ConstraintSolver to solve inverse kinematics by using a weighted least norm
class UnifiedJointLimitSingularitySolver : public ConstraintSolver<>
{
    public:
        UnifiedJointLimitSingularitySolver(const TwistControllerParams& params,
                                const LimiterParams& limiter_params,
                                TaskStackController_t& task_stack_controller) :
                ConstraintSolver(params, limiter_params, task_stack_controller)
        {}

        virtual ~UnifiedJointLimitSingularitySolver()
        {}

        /**
         * Specific implementation of solve-method to solve IK problem with joint limit avoidance.
         * See base class ConstraintSolver for more details on params and returns.
         */
        virtual Eigen::MatrixXd solve(const Vector6d_t& in_cart_velocities,
                                      const JointStates& joint_states);

    private:
        /**
         * Virtual helper method that calculates a weighting for the Jacobian to adapt joint velocity calculation for given constraints.
         * @param q The current joint positions.
         * @param q_dot The current joint velocities.
         * @return Diagonal weighting matrix that adapts the Jacobian.
         */
        virtual Eigen::MatrixXd calculateWeighting(const Vector6d_t& in_cart_velocities, const JointStates& joint_states) const;
};

#endif  // COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_UNIFIED_JOINT_LIMIT_SINGULARITY_SOLVER_H
