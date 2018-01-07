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


#include <ros/ros.h>

#include "cob_twist_controller/constraint_solvers/solvers/unconstraint_solver.h"

/**
 * Implementation of a default solve-method for the inverse kinematics problem.
 * It calculates the pseudo-inverse of the Jacobian via the base implementation of calculatePinvJacobianBySVD.
 * With the pseudo-inverse the joint velocity vector is calculated.
 */
Eigen::MatrixXd UnconstraintSolver::solve(const Vector6d_t& in_cart_velocities,
                                          const JointStates& joint_states)
{
    Eigen::MatrixXd pinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd qdots_out = pinv * in_cart_velocities;
    return qdots_out;
}

