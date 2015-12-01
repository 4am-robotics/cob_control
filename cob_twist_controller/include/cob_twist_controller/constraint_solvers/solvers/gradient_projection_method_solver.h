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
 *   This header contains the description of the unconstraint solver
 *   Implements methods from constraint_solver_base
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_GRADIENT_PROJECTION_METHOD_SOLVER_H
#define COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_GRADIENT_PROJECTION_METHOD_SOLVER_H

#include <set>
#include "ros/ros.h"

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

#include "cob_twist_controller/constraints/constraint_base.h"
#include "cob_twist_controller/constraints/constraint.h"

class GradientProjectionMethodSolver : public ConstraintSolver<>
{
    public:
        GradientProjectionMethodSolver(const TwistControllerParams& params,
                                       const LimiterParams& limiter_params,
                                       TaskStackController_t& task_stack_controller) :
                ConstraintSolver(params, limiter_params, task_stack_controller)
        {}

        virtual ~GradientProjectionMethodSolver()
        {}

        /**
         * Specific implementation of solve-method to solve IK problem with constraints by using the GPM.
         * See base class ConstraintSolver for more details on params and returns.
         */
        virtual Eigen::MatrixXd solve(const Vector6d_t& in_cart_velocities,
                                      const JointStates& joint_states);
};

#endif  // COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_GRADIENT_PROJECTION_METHOD_SOLVER_H
