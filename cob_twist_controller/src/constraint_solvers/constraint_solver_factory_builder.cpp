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

#include "ros/ros.h"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/jntarray.hpp>
#include <boost/shared_ptr.hpp>
#include "cob_twist_controller/augmented_solver_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"
#include "cob_twist_controller/constraint_solvers/solvers/unconstraint_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/wln_joint_limit_avoidance_solver.h"
#include "cob_twist_controller/constraint_solvers/solvers/weighted_least_norm_solver.h"
#include "cob_twist_controller/constraint_solvers/constraint_solver_factory_builder.h"
#include "cob_twist_controller/damping_methods/damping.h"
#include "cob_twist_controller/constraint_solvers/factories/solver_factory.h"

/**
 * Out of the parameters generates a damping method (e.g. constant or manipulability) and calculates the damping factor.
 * Dependent on JLA active flag a JointLimitAvoidanceSolver or a UnconstraintSolver is generated to solve the IK problem.
 * The objects are generated for each solve-request. After calculation the objects are deleted.
 */
int8_t ConstraintSolverFactoryBuilder::calculateJointVelocities(AugmentedSolverParams &asParams,
                                                                        Matrix6Xd &jacobianData,
                                                                        const Eigen::VectorXd &inCartVelocities,
                                                                        const KDL::JntArray& q,
                                                                        const KDL::JntArray& last_q_dot,
                                                                        const Eigen::VectorXd& tracking_errors,
                                                                        Eigen::MatrixXd &outJntVelocities)
{
    outJntVelocities = Eigen::MatrixXd();
    boost::shared_ptr<DampingBase> db (DampingBuilder::create_damping(asParams, jacobianData));
    if(NULL == db)
    {
        ROS_ERROR("Returning NULL factory due to damping creation error.");
        return -1; // error
    }

    // ISolverFactory* sf = NULL;
    boost::shared_ptr<ISolverFactory> sf;
    switch(asParams.constraint)
    {
        case None:
            sf.reset(new SolverFactory<UnconstraintSolver>());
            break;
        case WLN:
            sf.reset(new SolverFactory<WeightedLeastNormSolver>());
            break;
        case WLN_JLA:
            sf.reset(new SolverFactory<WLN_JointLimitAvoidanceSolver>());
            break;
        default:
            ROS_ERROR("Returning NULL factory due to constraint solver creation error.");
            break;
    }

    if (NULL != sf)
    {
        outJntVelocities = sf->calculateJointVelocities(asParams,
                                                        jacobianData,
                                                        inCartVelocities,
                                                        q,
                                                        last_q_dot,
                                                        tracking_errors,
                                                        db);
    }
    else
    {
        return -2; // error: no valid selection for constraint
    }

    sf.reset();
    db.reset();

    return 0; // success
}
