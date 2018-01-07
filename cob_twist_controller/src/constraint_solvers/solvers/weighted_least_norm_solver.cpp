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


#include "cob_twist_controller/constraint_solvers/solvers/weighted_least_norm_solver.h"

/**
 * Specific implementation of the solve method using a weighted least norm.
 * This is done by calculation of a weighting which is dependent on inherited classes for the Jacobian.
 * Uses the base implementation of calculatePinvJacobianBySVD to calculate the pseudo-inverse (weighted) Jacobian.
 */
Eigen::MatrixXd WeightedLeastNormSolver::solve(const Vector6d_t& in_cart_velocities,
                                               const JointStates& joint_states)
{
    Eigen::MatrixXd W_WLN = this->calculateWeighting(joint_states);
    // for the following formulas see Chan paper ISSN 1042-296X [Page 288]
    Eigen::MatrixXd root_W_WLN =  W_WLN.cwiseSqrt();            // element-wise sqrt -> ok because diag matrix W^(1/2)
    Eigen::MatrixXd inv_root_W_WLN =  root_W_WLN.inverse();     // -> W^(-1/2)

    // SVD of JLA weighted Jacobian: Damping will be done later in calculatePinvJacobianBySVD for pseudo-inverse Jacobian with additional truncation etc.
    Eigen::MatrixXd weighted_jacobian = this->jacobian_data_ * inv_root_W_WLN;
    Eigen::MatrixXd pinv = pinv_calc_.calculate(this->params_, this->damping_, weighted_jacobian);

    // Take care: W^(1/2) * q_dot = weighted_pinv_J * x_dot -> One must consider the weighting!!!
    Eigen::MatrixXd qdots_out = inv_root_W_WLN * pinv * in_cart_velocities;
    return qdots_out;
}

/**
 * This function returns the identity as weighting matrix for base functionality.
 */
Eigen::MatrixXd WeightedLeastNormSolver::calculateWeighting(const JointStates& joint_states) const
{
    uint32_t cols = this->jacobian_data_.cols();
    Eigen::VectorXd weighting = Eigen::VectorXd::Ones(cols);
    return weighting.asDiagonal();
}
