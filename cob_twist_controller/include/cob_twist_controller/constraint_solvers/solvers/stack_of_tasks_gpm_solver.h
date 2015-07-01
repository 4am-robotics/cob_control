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
 *   This header contains the description of stack of tasks solver
 *   Implements methods from constraint_solver_base
 *
 ****************************************************************/
#ifndef STACK_OF_TASKS_SOLVER_2ND_H_
#define STACK_OF_TASKS_SOLVER_2ND_H_

#include <set>
#include "ros/ros.h"

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"


#include "cob_twist_controller/constraints/constraint_base.h"
#include "cob_twist_controller/constraints/constraint.h"

class StackOfTasksSolver2nd : public ConstraintSolver<>
{
    public:
        StackOfTasksSolver2nd(const TwistControllerParams& params)
                           : ConstraintSolver(params),
                             last_min_distance_(-1.0),
                             last_cycle_time_(-1.0)
        {
            last_in_cart_velocities_ = t_Vector6d::Zero();
            last_jac_ = t_Matrix76d::Zero();

            ROS_INFO_STREAM("StackOfTasksSolver2nd created!!!");
        }

        virtual ~StackOfTasksSolver2nd()
        {
            this->clearConstraints();
        }

        /**
         * Specific implementation of solve-method to solve IK problem with constraints by using the GPM.
         * See base class ConstraintSolver for more details on params and returns.
         */
        virtual Eigen::MatrixXd solve(const t_Vector6d& in_cart_velocities,
                                      const JointStates& joint_states);

        /**
         * Set all created constraints in a (priorized) set.
         * @param constraints: All constraints ordered according to priority.
         */
        virtual void setConstraints(std::set<tConstraintBase>& constraints)
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

    protected:
        double last_min_distance_;
        double last_cycle_time_;
        t_Vector6d last_in_cart_velocities_;
        t_Matrix76d last_jac_;

        /// set inserts sorted (default less operator); if element has already been added it returns an iterator on it.
        std::set<tConstraintBase> constraints_;
};

#endif /* STACK_OF_TASKS_SOLVER_2ND_H_ */
