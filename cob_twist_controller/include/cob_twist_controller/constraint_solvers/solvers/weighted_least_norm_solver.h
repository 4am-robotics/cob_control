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
#ifndef WEIGHTED_LEAST_NORM_SOLVER_H_
#define WEIGHTED_LEAST_NORM_SOLVER_H_

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

/// Implementation of ConstraintSolver to solve inverse kinematics by using a weighted least norm
class WeightedLeastNormSolver : public ConstraintSolver<>
{
    public:

        /**
         * Specific implementation of solve-method to solve IK problem with joint limit avoidance.
         * See base class ConstraintSolver for more details on params and returns.
         */
        virtual Eigen::MatrixXd solve(const Vector6d_t& in_cart_velocities,
                                      const JointStates& joint_states);

        WeightedLeastNormSolver(const TwistControllerParams& params, TaskStackController_t& task_stack_controller)
                                  : ConstraintSolver(params, task_stack_controller)
        {

        }

        virtual ~WeightedLeastNormSolver()
        {

        }


    private:

        /**
         * Virtual helper method that calculates a weighting for the Jacobian to adapt joint velocity calculation for given constraints.
         * @param q The current joint positions.
         * @param q_dot The current joint velocities.
         * @return Diagonal weighting matrix that adapts the Jacobian.
         */
        virtual Eigen::MatrixXd calculateWeighting(const JointStates& joint_states) const;
};

#endif /* WEIGHTED_LEAST_NORM_SOLVER_H_ */
