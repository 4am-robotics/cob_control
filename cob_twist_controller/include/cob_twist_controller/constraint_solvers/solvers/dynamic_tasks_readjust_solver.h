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
#ifndef DYNAMIC_TASKS_READJUST_SOLVER_H_
#define DYNAMIC_TASKS_READJUST_SOLVER_H_

#include <set>
#include <ros/ros.h>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

#include "cob_twist_controller/constraints/constraint_base.h"
#include "cob_twist_controller/constraints/constraint.h"

class DynamicTasksReadjustSolver : public ConstraintSolver<>
{
    public:
        DynamicTasksReadjustSolver(const TwistControllerParams& params, TaskStackController_t& task_stack_controller)
                           : ConstraintSolver(params, task_stack_controller), in_cart_vel_damping_(1.0)
        {
        }

        virtual ~DynamicTasksReadjustSolver()
        {
            this->clearConstraints();
        }

        /**
         * Specific implementation of solve-method to solve IK problem with constraints by using the GPM.
         * See base class ConstraintSolver for more details on params and returns.
         */
        virtual Eigen::MatrixXd solve(const Vector6d_t& in_cart_velocities,
                                      const JointStates& joint_states);

        /**
         * Set all created constraints in a (priorized) set.
         * @param constraints: All constraints ordered according to priority.
         */
        virtual void setConstraints(std::set<ConstraintBase_t>& constraints)
        {
            this->constraints_ = constraints;
        }

        /**
         * Calls destructor on all objects and clears the set
         */
        void clearConstraints()
        {
            this->constraints_.clear();
        }

        /**
         * Process the state of the constraint and update the sum_of_gradient.
         */
        void processState(std::set<ConstraintBase_t>::iterator& it,
                          const Eigen::MatrixXd& projector,
                          const Eigen::MatrixXd& particular_solution,
                          Eigen::VectorXd& sum_of_gradient);

    protected:

        /// set inserts sorted (default less operator); if element has already been added it returns an iterator on it.
        std::set<ConstraintBase_t> constraints_;


        double in_cart_vel_damping_;
};

#endif /* DYNAMIC_TASKS_READJUST_SOLVER_H_ */
