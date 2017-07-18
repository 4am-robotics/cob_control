/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <set>
#include <ros/ros.h>

#include <cob_twist_controller/constraint_solvers/constraint_solver_factory.h>
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"
#include "cob_twist_controller/constraint_solvers/solvers/unconstraint_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/wln_joint_limit_avoidance_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/weighted_least_norm_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/gradient_projection_method_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/task_priority_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/stack_of_tasks_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/unified_joint_limit_singularity_solver.h"

#include "cob_twist_controller/damping_methods/damping.h"
#include "cob_twist_controller/constraints/constraint.h"

/**
 * Out of the parameters generates a damping method (e.g. constant or manipulability) and calculates the damping factor.
 * Dependent on JLA active flag a JointLimitAvoidanceSolver or a UnconstraintSolver is generated to solve the IK problem.
 * The objects are generated for each solve-request. After calculation the objects are deleted.
 */
int8_t ConstraintSolverFactory::calculateJointVelocities(Matrix6Xd_t& jacobian_data,
                                                         const Vector6d_t& in_cart_velocities,
                                                         const JointStates& joint_states,
                                                         Eigen::MatrixXd& out_jnt_velocities)
{
    out_jnt_velocities = Eigen::MatrixXd::Zero(joint_states.current_q_dot_.rows(),
                                               joint_states.current_q_dot_.columns());

    if (NULL == this->damping_method_)
    {
        return -1;  // damping method not initialized
    }
    else if (NULL == this->solver_factory_)
    {
        return -2;  // solver factory not initialized
    }
    else
    {
        // everything seems to be alright!
    }

    out_jnt_velocities = this->solver_factory_->calculateJointVelocities(jacobian_data,
                                                                         in_cart_velocities,
                                                                         joint_states,
                                                                         this->damping_method_,
                                                                         this->constraints_);

    return 0;   // success
}

/**
 * Given a proper constraint_type a corresponding SolverFactory is generated and returned.
 */
bool ConstraintSolverFactory::getSolverFactory(const TwistControllerParams& params,
                                               const LimiterParams& limiter_params,
                                               boost::shared_ptr<ISolverFactory>& solver_factory,
                                               TaskStackController_t& task_stack_controller)
{
    switch (params.solver)
    {
        case DEFAULT_SOLVER:
            solver_factory.reset(new SolverFactory<UnconstraintSolver>(params, limiter_params, task_stack_controller));
            break;
        case WLN:
            switch (params.constraint_jla)
            {
                case JLA_ON:
                    solver_factory.reset(new SolverFactory<WLN_JointLimitAvoidanceSolver>(params, limiter_params, task_stack_controller));
                break;

                case JLA_OFF:
                    solver_factory.reset(new SolverFactory<WeightedLeastNormSolver>(params, limiter_params, task_stack_controller));
                break;
            }
            break;
        case UNIFIED_JLA_SA:
            solver_factory.reset(new SolverFactory<UnifiedJointLimitSingularitySolver>(params, limiter_params, task_stack_controller));
            break;
        case GPM:
            solver_factory.reset(new SolverFactory<GradientProjectionMethodSolver>(params, limiter_params, task_stack_controller));
            break;
        case STACK_OF_TASKS:
            solver_factory.reset(new SolverFactory<StackOfTasksSolver>(params, limiter_params, task_stack_controller));
            break;
        case TASK_2ND_PRIO:
            solver_factory.reset(new SolverFactory<TaskPrioritySolver>(params, limiter_params, task_stack_controller));
            break;
        default:
            ROS_ERROR("Returning NULL factory due to constraint solver creation error. There is no solver method for %d implemented.",
                      params.solver);
            return false;
    }

    return true;
}

int8_t ConstraintSolverFactory::resetAll(const TwistControllerParams& params, const LimiterParams& limiter_params)
{
    this->damping_method_.reset(DampingBuilder::createDamping(params));
    if (NULL == this->damping_method_)
    {
        ROS_ERROR("Returning NULL due to damping creation error.");
        return -1;  // error
    }

    this->constraints_.clear();
    this->constraints_ = ConstraintsBuilder_t::createConstraints(params,
                                                                 limiter_params,
                                                                 this->jnt_to_jac_,
                                                                 this->fk_solver_vel_,
                                                                 this->data_mediator_);

    for (std::set<ConstraintBase_t>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        ROS_DEBUG_STREAM((*it)->getTaskId());
    }

    if (!ConstraintSolverFactory::getSolverFactory(params, limiter_params, this->solver_factory_, this->task_stack_controller_))
    {
        return -2;
    }

    return 0;
}
