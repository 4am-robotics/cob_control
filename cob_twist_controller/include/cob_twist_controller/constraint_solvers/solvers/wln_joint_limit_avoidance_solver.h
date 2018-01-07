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


#ifndef COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_WLN_JOINT_LIMIT_AVOIDANCE_SOLVER_H
#define COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_WLN_JOINT_LIMIT_AVOIDANCE_SOLVER_H

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/weighted_least_norm_solver.h"

/// Implementation of ConstraintSolver to solve inverse kinematics with joint limit avoidance
/// Uses solve method of the WeightedLeastNormSolver
class WLN_JointLimitAvoidanceSolver : public WeightedLeastNormSolver
{
    public:
        WLN_JointLimitAvoidanceSolver(const TwistControllerParams& params,
                                      const LimiterParams& limiter_params,
                                      TaskStackController_t& task_stack_controller) :
                WeightedLeastNormSolver(params, limiter_params, task_stack_controller)
        {}

        virtual ~WLN_JointLimitAvoidanceSolver()
        {}

    private:
        /**
         * Helper method that calculates a weighting for the Jacobian to adapt the impact on joint velocities.
         * Overridden from base class WLNSolver
         * @param q The current joint positions.
         * @param q_dot The current joint velocities.
         * @return Diagonal weighting matrix that adapts the Jacobian.
         */
        virtual Eigen::MatrixXd calculateWeighting(const JointStates& joint_states) const;
};

#endif  // COB_TWIST_CONTROLLER_CONSTRAINT_SOLVERS_SOLVERS_WLN_JOINT_LIMIT_AVOIDANCE_SOLVER_H
