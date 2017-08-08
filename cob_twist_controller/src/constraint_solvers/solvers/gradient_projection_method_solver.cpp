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


#include <set>
#include <cmath>

#include "cob_twist_controller/constraint_solvers/solvers/gradient_projection_method_solver.h"

/**
 * Solve the inverse differential kinematics equation by using GPM.
 * In addtion to the partial solution q_dot = J^+ * v the homogeneous solution (I - J^+ * J) q_dot_0 is calculated.
 * The q_dot_0 results from the sum of the constraint cost function gradients. The terms of the sum are weighted with a factor k_H separately.
 */
Eigen::MatrixXd GradientProjectionMethodSolver::solve(const Vector6d_t& in_cart_velocities,
                                                      const JointStates& joint_states)
{
    Eigen::MatrixXd damped_pinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::MatrixXd pinv = pinv_calc_.calculate(this->jacobian_data_);

    Eigen::MatrixXd particular_solution = damped_pinv * in_cart_velocities;

    // Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(damped_pinv.rows(), this->jacobian_data_.cols());
    // Eigen::MatrixXd projector = ident - damped_pinv * this->jacobian_data_;
    Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(pinv.rows(), this->jacobian_data_.cols());
    Eigen::MatrixXd projector = ident - pinv * this->jacobian_data_;

    Eigen::MatrixXd homogeneous_solution = Eigen::MatrixXd::Zero(particular_solution.rows(), particular_solution.cols());
    KDL::JntArrayVel predict_jnts_vel(joint_states.current_q_.rows());

    for (std::set<ConstraintBase_t>::iterator it = this->constraints_.begin(); it != this->constraints_.end(); ++it)
    {
        ROS_DEBUG_STREAM("task id: " << (*it)->getTaskId());
        (*it)->update(joint_states, predict_jnts_vel, this->jacobian_data_);
        Eigen::VectorXd q_dot_0 = (*it)->getPartialValues();
        Eigen::MatrixXd tmp_projection = projector * q_dot_0;
        double activation_gain = (*it)->getActivationGain();  // contribution of the homo. solution to the part. solution
        double constraint_k_H = (*it)->getSelfMotionMagnitude(particular_solution, tmp_projection);  // gain of homogenous solution (if active)
        homogeneous_solution += (constraint_k_H * activation_gain * tmp_projection);
    }

    Eigen::MatrixXd qdots_out = particular_solution + this->params_.k_H * homogeneous_solution;  // weighting with k_H is done in loop

    // //DEBUG: for verification of nullspace projection
    // std::stringstream ss_part;
    // ss_part << "particular_solution: ";
    // for(unsigned int i=0; i<particular_solution.rows(); i++)
    // {   ss_part << particular_solution(i,0) << " , ";    }
    // ROS_INFO_STREAM(ss_part.str());
    // std::stringstream ss_hom;
    // ss_hom << "homogeneous_solution: ";
    // for(unsigned int i=0; i<homogeneous_solution.rows(); i++)
    // {   ss_hom << homogeneous_solution(i,0) << " , ";    }
    // ROS_INFO_STREAM(ss_hom.str());
    // Vector6d_t resultingCartVelocities = this->jacobian_data_ * qdots_out;
    // std::stringstream ss_fk;
    // ss_fk << "resultingCartVelocities: ";
    // for(unsigned int i=0; i<resultingCartVelocities.rows(); i++)
    // {   ss_fk << resultingCartVelocities(i,0) << " , ";    }
    // ROS_INFO_STREAM(ss_fk.str());

    return qdots_out;
}
