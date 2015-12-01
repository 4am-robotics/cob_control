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
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This header contains the description of an priority based
 *   task solver with additional gradient projection.
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_STACK_OF_TASKS_SOLVER_H
#define COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_STACK_OF_TASKS_SOLVER_H

#include <set>
#include <ros/ros.h>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

#include "cob_twist_controller/constraints/constraint_base.h"
#include "cob_twist_controller/constraints/constraint.h"

#define START_CNT 40.0

class StackOfTasksSolver : public ConstraintSolver<>
{
    public:
        StackOfTasksSolver(const TwistControllerParams& params,
                           const LimiterParams& limiter_params,
                           TaskStackController_t& task_stack_controller) :
                ConstraintSolver(params, limiter_params, task_stack_controller)
        {
            this->last_time_ = ros::Time::now();
            this->global_constraint_state_ = NORMAL;
            this->in_cart_vel_damping_ = 1.0;
        }

        virtual ~StackOfTasksSolver()
        {}

        /**
         * Specific implementation of solve-method to solve IK problem with constraints by using the GPM.
         * See base class ConstraintSolver for more details on params and returns.
         */
        virtual Eigen::MatrixXd solve(const Vector6d_t& in_cart_velocities,
                                      const JointStates& joint_states);

        /**
         * Process the state of the constraint and update the sum_of_gradient.
         */
        void processState(std::set<ConstraintBase_t>::iterator& it,
                          const Eigen::MatrixXd& projector,
                          const Eigen::MatrixXd& particular_solution,
                          double inv_sum_of_prios,
                          Eigen::VectorXd& sum_of_gradient);

    protected:
        ros::Time last_time_;
        EN_ConstraintStates global_constraint_state_;
        double in_cart_vel_damping_;
};

#endif  // COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_STACK_OF_TASKS_SOLVER_H
