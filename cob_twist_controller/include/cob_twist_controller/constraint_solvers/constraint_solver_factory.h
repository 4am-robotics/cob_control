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

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/factories/solver_factory.h"
#include "cob_twist_controller/callback_data_mediator.h"

/// Static class providing a single method for creation of damping method, solver and starting the solving of the IK problem.
class ConstraintSolverFactory
{
    public:

        /**
         * Ctor of ConstraintSolverFactoryBuilder.
         * @param data_mediator: Reference to an callback data mediator.
         * @param jnt_to_jac: Reference to an joint to Jacobian solver.
         */
        ConstraintSolverFactory(CallbackDataMediator& data_mediator, KDL::ChainJntToJacSolver& jnt_to_jac) :
            data_mediator_(data_mediator), jnt_to_jac_(jnt_to_jac)
        {
            this->solver_factory_.reset();
            this->damping_method_.reset();
        }

        ~ConstraintSolverFactory()
        {
            this->solver_factory_.reset();
            this->damping_method_.reset();
        }

        /**
         * Calculation of new joint velocities according to current joint positions and cartesian velocities.
         * @param params References the inv. diff. kin. solver parameters.
         * @param jacobian_data References the current Jacobian (matrix data only).
         * @param in_cart_velocities The input velocities vector (in cartesian space).
         * @param joint_states The joint states and history.
         * @param out_jnt_velocities The calculated joint velocities as output reference.
         * @return The calculated new joint velocities in (m x 1)-Matrix.
         */
        int8_t calculateJointVelocities(t_Matrix6Xd& jacobian_data,
                                        const t_Vector6d& in_cart_velocities,
                                        const JointStates& joint_states,
                                        Eigen::MatrixXd& out_jnt_velocities);

        /**
         * Given a constraint_type create a solver_factory instance and return it.
         * In case of an error false will be returned.
         * @param constraint_type: Enum value of the constraint.
         * @param solver_factory: Reference of a shared pointer to be filled.
         */
        static bool getSolverFactory(const TwistControllerParams& params,
                                     boost::shared_ptr<ISolverFactory>& solver_factory);

        int8_t resetAll(const TwistControllerParams& params);

    private:
        CallbackDataMediator& data_mediator_;
        KDL::ChainJntToJacSolver& jnt_to_jac_;
        boost::shared_ptr<ISolverFactory> solver_factory_;
        boost::shared_ptr<DampingBase> damping_method_;
        std::set<tConstraintBase> constraints_;
};

#endif /* CONSTRAINT_SOLVER_FACTORY_BUILDER_H_ */
