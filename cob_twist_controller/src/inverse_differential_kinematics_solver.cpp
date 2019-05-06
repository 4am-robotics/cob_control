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
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/chainfksolvervel_recursive.hpp>

#include "cob_twist_controller/inverse_differential_kinematics_solver.h"

/**
 * Solve the inverse kinematics problem at the first order differential level.
 */
int InverseDifferentialKinematicsSolver::CartToJnt(const JointStates& joint_states,
                                                   const KDL::Twist& v_in,
                                                   KDL::JntArray& qdot_out)
{
    // ROS_INFO_STREAM("joint_states.current_q_: " << joint_states.current_q_.rows());
    int8_t retStat = -1;

    /// Let the ChainJntToJacSolver calculate the jacobian "jac_chain" for the current joint positions "q_in"
    KDL::Jacobian jac_chain(chain_.getNrOfJoints());
    jnt2jac_.JntToJac(joint_states.current_q_, jac_chain);
    // ROS_INFO_STREAM("jac_chain.rows: " << jac_chain.rows() << ", jac_chain.columns: " << jac_chain.columns());

    JointStates joint_states_full = this->kinematic_extension_->adjustJointStates(joint_states);
    // ROS_INFO_STREAM("joint_states_full.current_q_: " << joint_states_full.current_q_.rows());

    /// append columns to Jacobian in order to reflect additional DoFs of kinematical extension
    KDL::Jacobian jac_full = this->kinematic_extension_->adjustJacobian(jac_chain);
    // ROS_INFO_STREAM("jac_full.rows: " << jac_full.rows() << ", jac_full.columns: " << jac_full.columns());

    /// apply input limiters for limiting Cartesian velocities (input Twist)
    Vector6d_t v_in_vec;
    KDL::Twist v_temp;
    v_temp = this->limiters_->enforceLimits(v_in);
    tf::twistKDLToEigen(v_temp, v_in_vec);

    Eigen::MatrixXd qdot_out_vec;
    retStat = constraint_solver_factory_.calculateJointVelocities(jac_full.data,
                                                                  v_in_vec,
                                                                  joint_states_full,
                                                                  qdot_out_vec);

    /// convert output
    KDL::JntArray qdot_out_full(jac_full.columns());
    for (int i = 0; i < jac_full.columns(); i++)
    {
        qdot_out_full(i) = qdot_out_vec(i);
        // ROS_INFO_STREAM("qdot_out_full " << i << ": " << qdot_out_full(i));
    }
    // ROS_INFO_STREAM("qdot_out_full.rows: " << qdot_out_full.rows());

    /// output limiters shut be applied here in order to be able to consider the additional DoFs within "AllLimit", too
    qdot_out_full = this->limiters_->enforceLimits(qdot_out_full, joint_states_full.current_q_);

    // ROS_INFO_STREAM("qdot_out_full.rows enforced: " << qdot_out_full.rows());
    // for (int i = 0; i < jac_full.columns(); i++)
    // {
    //     ROS_INFO_STREAM("i: " << i << ", qdot_out_full: " << qdot_out_full(i));
    // }

    /// process result for kinematical extension
    this->kinematic_extension_->processResultExtension(qdot_out_full);

    /// then qdot_out shut be resized to contain only the chain_qdot_out's again
    for (int i = 0; i < jac_chain.columns(); i++)
    {
        qdot_out(i) = qdot_out_full(i);
    }

    return retStat;
}

bool InverseDifferentialKinematicsSolver::resetAll(TwistControllerParams params)
{
    this->params_ = params;

    this->kinematic_extension_.reset(KinematicExtensionBuilder::createKinematicExtension(this->params_));
    if (this->kinematic_extension_ == NULL) { return false; }
    this->limiter_params_ = this->kinematic_extension_->adjustLimiterParams(this->params_.limiter_params);

    this->limiters_.reset(new LimiterContainer(this->limiter_params_));
    this->limiters_->init();

    this->task_stack_controller_.clearAllTasks();
    if (0 != this->constraint_solver_factory_.resetAll(this->params_, this->limiter_params_))  // params member as reference!!! else process will die!
    {
        ROS_ERROR("Failed to reset IDK constraint solver after dynamic_reconfigure.");
        return false;
    }
    return true;
}
