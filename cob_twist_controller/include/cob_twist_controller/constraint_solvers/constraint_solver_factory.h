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


#ifndef COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_CONSTRAINT_SOLVER_FACTORY_H
#define COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_CONSTRAINT_SOLVER_FACTORY_H

#include <set>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
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
        ConstraintSolverFactory(CallbackDataMediator& data_mediator,
                                KDL::ChainJntToJacSolver& jnt_to_jac,
                                KDL::ChainFkSolverVel_recursive& fk_solver_vel,
                                TaskStackController_t& task_stack_controller) :
            data_mediator_(data_mediator),
            jnt_to_jac_(jnt_to_jac),
            task_stack_controller_(task_stack_controller),
            fk_solver_vel_(fk_solver_vel)
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
        int8_t calculateJointVelocities(Matrix6Xd_t& jacobian_data,
                                        const Vector6d_t& in_cart_velocities,
                                        const JointStates& joint_states,
                                        Eigen::MatrixXd& out_jnt_velocities);

        /**
         * Given a constraint_type create a solver_factory instance and return it.
         * In case of an error false will be returned.
         * @param constraint_type: Enum value of the constraint.
         * @param solver_factory: Reference of a shared pointer to be filled.
         */
        static bool getSolverFactory(const TwistControllerParams& params,
                                     const LimiterParams& limiter_params,
                                     boost::shared_ptr<ISolverFactory>& solver_factory,
                                     TaskStackController_t& task_stack_controller);

        int8_t resetAll(const TwistControllerParams& params, const LimiterParams& limiter_params);

    private:
        CallbackDataMediator& data_mediator_;
        KDL::ChainJntToJacSolver& jnt_to_jac_;
        KDL::ChainFkSolverVel_recursive& fk_solver_vel_;

        boost::shared_ptr<ISolverFactory> solver_factory_;
        boost::shared_ptr<DampingBase> damping_method_;
        std::set<ConstraintBase_t> constraints_;
        TaskStackController_t& task_stack_controller_;
};

#endif  // COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_CONSTRAINT_SOLVER_FACTORY_H
