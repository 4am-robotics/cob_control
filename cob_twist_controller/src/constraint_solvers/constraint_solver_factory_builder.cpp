/*
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
 *   This header contains the description of a class providing a static method to create constraint solver factory objects.
 *
 ****************************************************************/

#include <ros/ros.h>

#include "cob_twist_controller/constraint_solvers/constraint_solver_factory_builder.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"
#include "cob_twist_controller/constraint_solvers/solvers/unconstraint_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/wln_joint_limit_avoidance_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/weighted_least_norm_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/gradient_projection_method_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/stack_of_tasks_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/stack_of_tasks_solver_2nd.h"
#include "cob_twist_controller/constraint_solvers/solvers/task_priority_solver.h"

#include "cob_twist_controller/damping_methods/damping.h"

#include "cob_twist_controller/constraints/constraint.h"

/**
 * Out of the parameters generates a damping method (e.g. constant or manipulability) and calculates the damping factor.
 * Dependent on JLA active flag a JointLimitAvoidanceSolver or a UnconstraintSolver is generated to solve the IK problem.
 * The objects are generated for each solve-request. After calculation the objects are deleted.
 */
int8_t ConstraintSolverFactoryBuilder::calculateJointVelocities(InvDiffKinSolverParams &params,
                                                                t_Matrix6Xd &jacobian_data,
                                                                const t_Vector6d &in_cart_velocities,
                                                                const JointStates& joint_states,
                                                                Eigen::MatrixXd &out_jnt_velocities)
{
    out_jnt_velocities = Eigen::MatrixXd::Zero(joint_states.current_q_dot_.rows(),
                                               joint_states.current_q_dot_.columns());
    boost::shared_ptr<DampingBase> db (DampingBuilder::create_damping(params, jacobian_data));
    if(NULL == db)
    {
        ROS_ERROR("Returning NULL factory due to damping creation error.");
        return -1; // error
    }

    std::set<tConstraintBase> constraints = ConstraintsBuilder<>::createConstraints(params,
                                                                                    joint_states,
                                                                                    jacobian_data,
                                                                                    this->jnt_to_jac_,
                                                                                    this->data_mediator_);

    boost::shared_ptr<ISolverFactory> sf;
    if (!ConstraintSolverFactoryBuilder::getSolverFactory(params.constraint, sf))
    {
        return -2; // error: no valid selection for constraint
    }

    out_jnt_velocities = sf->calculateJointVelocities(params,
                                                    jacobian_data,
                                                    in_cart_velocities,
                                                    joint_states,
                                                    db,
                                                    constraints);

    sf.reset();
    db.reset();

    return 0; // success
}

/**
 * Given a proper constraint_type a corresponding SolverFactory is generated and returned.
 */
bool ConstraintSolverFactoryBuilder::getSolverFactory(uint32_t constraint_type,
                                                      boost::shared_ptr<ISolverFactory>& solver_factory)
{
    switch(constraint_type)
    {
        case None:
            solver_factory.reset(new SolverFactory<UnconstraintSolver>());
            break;
        case WLN:
            solver_factory.reset(new SolverFactory<WeightedLeastNormSolver>());
            break;
        case WLN_JLA:
            solver_factory.reset(new SolverFactory<WLN_JointLimitAvoidanceSolver>());
            break;
        case GPM_JLA:
        case GPM_JLA_MID:
        case GPM_CA:
            solver_factory.reset(new SolverFactory<GradientProjectionMethodSolver>());
            break;
        case TASK_STACK:
            solver_factory.reset(new SolverFactory<StackOfTasksSolver>());
            break;
        case TASK_STACK_2ND:
            solver_factory.reset(new SolverFactory<StackOfTasksSolver2nd>());
            break;
        case TASK_PRIO:
            solver_factory.reset(new SolverFactory<TaskPrioritySolver>());
            break;
        default:
            ROS_ERROR("Returning NULL factory due to constraint solver creation error. There is no solver method for %d implemented.",
                      constraint_type);
            return false;
    }

    return true;
}
