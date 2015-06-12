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
#ifndef CONSTRAINT_SOLVER_FACTORY_BUILDER_H_
#define CONSTRAINT_SOLVER_FACTORY_BUILDER_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/jntarray.hpp>
#include <boost/shared_ptr.hpp>

#include "cob_twist_controller/augmented_solver_data_types.h"
#include "cob_twist_controller/constraint_solvers/factories/solver_factory.h"
#include "cob_twist_controller/callback_data_mediator.h"

/// Static class providing a single method for creation of damping method, solver and starting the solving of the IK problem.
class ConstraintSolverFactoryBuilder
{
    public:

        /**
         * Ctor of ConstraintSolverFactoryBuilder.
         * @param data_mediator: Reference to an callback data mediator.
         * @param jnt_to_jac: Reference to an joint to Jacobian solver.
         */
        ConstraintSolverFactoryBuilder(CallbackDataMediator& data_mediator, KDL::ChainJntToJacSolver& jnt_to_jac) :
            data_mediator_(data_mediator), jnt_to_jac_(jnt_to_jac)
        {

        }

        ~ConstraintSolverFactoryBuilder() {}

        /**
         * Calculation of new joint velocities according to current joint positions and cartesian velocities.
         * @param asParams References the augmented solver parameters.
         * @param jacobianData References the current Jacobian (matrix data only).
         * @param inCartVelocities The input velocities vector (in cartesian space).
         * @param q The current joint positions.
         * @param last_q_dot The last joint velocities.
         * @param outJntVelocities The calculated joint velocities as output reference.
         * @return The calculated new joint velocities in (m x 1)-Matrix.
         */
        int8_t calculateJointVelocities(AugmentedSolverParams &asParams,
                                        Matrix6Xd &jacobianData,
                                        const Vector6d &inCartVelocities,
                                        const KDL::JntArray& q,
                                        const KDL::JntArray& last_q_dot,
                                        Eigen::MatrixXd &outJntVelocities);

        /**
         * Given a constraint_type create a solver_factory instance and return it.
         * In case of an error false will be returned.
         * @param constraint_type: Enum value of the constraint.
         * @param solver_factory: Reference of a shared pointer to be filled.
         */
        static bool getSolverFactory(uint32_t constraint_type,
                                     boost::shared_ptr<ISolverFactory>& solver_factory);

    private:
        CallbackDataMediator& data_mediator_;
        KDL::ChainJntToJacSolver& jnt_to_jac_;
};

#endif /* CONSTRAINT_SOLVER_FACTORY_BUILDER_H_ */
