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


#ifndef COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_WEIGHTED_LEAST_NORM_SOLVER_H
#define COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_WEIGHTED_LEAST_NORM_SOLVER_H

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

/// Implementation of ConstraintSolver to solve inverse kinematics by using a weighted least norm
class WeightedLeastNormSolver : public ConstraintSolver<>
{
    public:
        WeightedLeastNormSolver(const TwistControllerParams& params,
                                const LimiterParams& limiter_params,
                                TaskStackController_t& task_stack_controller) :
                ConstraintSolver(params, limiter_params, task_stack_controller)
        {}

        virtual ~WeightedLeastNormSolver()
        {}

        /**
         * Specific implementation of solve-method to solve IK problem with joint limit avoidance.
         * See base class ConstraintSolver for more details on params and returns.
         */
        virtual Eigen::MatrixXd solve(const Vector6d_t& in_cart_velocities,
                                      const JointStates& joint_states);

    private:
        /**
         * Virtual helper method that calculates a weighting for the Jacobian to adapt joint velocity calculation for given constraints.
         * @param q The current joint positions.
         * @param q_dot The current joint velocities.
         * @return Diagonal weighting matrix that adapts the Jacobian.
         */
        virtual Eigen::MatrixXd calculateWeighting(const JointStates& joint_states) const;
};

#endif  // COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_WEIGHTED_LEAST_NORM_SOLVER_H
