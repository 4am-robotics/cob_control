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


#ifndef COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_CONSTRAINT_SOLVER_BASE_H
#define COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_CONSTRAINT_SOLVER_BASE_H

#include <set>
#include <Eigen/Core>
#include <kdl/jntarray.hpp>
#include <boost/shared_ptr.hpp>
#include <cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h>
#include "cob_twist_controller/damping_methods/damping_base.h"
#include "cob_twist_controller/constraints/constraint_base.h"
#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/task_stack/task_stack_controller.h"

/// Base class for solvers, defining interface methods.
template <typename PINV = PInvBySVD>
class ConstraintSolver
{
    public:
        /**
         * The interface method to solve the inverse kinematics problem. Has to be implemented in inherited classes.
         * @param in_cart_velocities The input velocities vector (in cartesian space).
         * @param joint_states The joint states with history.
         * @return The calculated new joint velocities.
         */
        virtual Eigen::MatrixXd solve(const Vector6d_t& in_cart_velocities,
                                      const JointStates& joint_states) = 0;

        /**
         * Inline method to set the damping
         * @param damping The new damping
         */
        inline void setDamping(boost::shared_ptr<DampingBase>& damping)
        {
            this->damping_ = damping;
        }

        /**
         * Set all created constraints in a (priorized) set.
         * @param constraints: All constraints ordered according to priority.
         */
        inline void setConstraints(std::set<ConstraintBase_t>& constraints)
        {
            this->constraints_.clear();
            this->constraints_ = constraints;
        }

        /**
         * Calls destructor on all objects and clears the set
         */
        inline void clearConstraints()
        {
            this->constraints_.clear();
        }

        /**
         * Method to initialize the solver if necessary
         */
        virtual void setJacobianData(const Matrix6Xd_t& jacobian_data)
        {
            this->jacobian_data_ = jacobian_data;
        }

        virtual ~ConstraintSolver()
        {
            this->clearConstraints();
        }

        ConstraintSolver(const TwistControllerParams& params,
                         const LimiterParams& limiter_params,
                         TaskStackController_t& task_stack_controller) :
                params_(params),
                limiter_params_(limiter_params),
                task_stack_controller_(task_stack_controller)
        {}

    protected:
        /// set inserts sorted (default less operator); if element has already been added it returns an iterator on it.
        std::set<ConstraintBase_t> constraints_;  /// Set of constraints.
        const TwistControllerParams& params_;  /// References the inv. diff. kin. solver parameters.
        const LimiterParams& limiter_params_;  /// References the limiter parameters (up-to-date according to KinematicExtension).
        Matrix6Xd_t jacobian_data_;  /// References the current Jacobian (matrix data only).
        boost::shared_ptr<DampingBase> damping_;  /// The currently set damping method.
        PINV pinv_calc_;  /// An instance that helps solving the inverse of the Jacobian.
        TaskStackController_t& task_stack_controller_;  /// Reference to the task stack controller.
};

#endif  // COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_CONSTRAINT_SOLVER_BASE_H
