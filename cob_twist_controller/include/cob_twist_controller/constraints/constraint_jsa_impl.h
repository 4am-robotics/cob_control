/*!
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
 * \date Date of creation: May, 2015
 *
 * \brief
 *   Implementation of several constraints
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_JSA_IMPL_H
#define COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_JSA_IMPL_H

#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include "cob_twist_controller/utils/chainjnttojacdotsolver.hpp"

#include "cob_twist_controller/constraints/constraint.h"
#include "cob_twist_controller/constraints/constraint_params.h"

#include "cob_twist_controller/damping_methods/damping.h"
#include "cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h"

#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/NumericalDiff>
#include <eigen_conversions/eigen_kdl.h>

/* BEGIN JointLimitAvoidance ************************************************************************************/
template <typename T_PARAMS, typename PRIO>
Task_t JointSingularityAvoidance<T_PARAMS, PRIO>::createTask()
{
    Eigen::MatrixXd cost_func_jac = this->getTaskJacobian();
    Eigen::VectorXd derivs = this->getTaskDerivatives();
    Task_t task(this->getPriority(),
                this->getTaskId(),
                cost_func_jac,
                derivs,
                this->getType());

    task.tcp_ = this->adaptDampingParamsForTask(this->constraint_params_.tc_params_.damping_jla);
    task.db_ = boost::shared_ptr<DampingBase>(DampingBuilder::createDamping(task.tcp_));
    return task;
}

template <typename T_PARAMS, typename PRIO>
std::string JointSingularityAvoidance<T_PARAMS, PRIO>::getTaskId() const
{
    std::ostringstream oss;
    oss << this->member_inst_cnt_;
    oss << "_Joint#";
    oss << this->constraint_params_.joint_idx_;
    oss << "_";
    oss << this->priority_;
    std::string taskid = "JointSingularityAvoidance_" + oss.str();
    return taskid;
}

template <typename T_PARAMS, typename PRIO>
Eigen::MatrixXd JointSingularityAvoidance<T_PARAMS, PRIO>::getTaskJacobian() const
{
    return this->partial_values_.transpose();
}

template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointSingularityAvoidance<T_PARAMS, PRIO>::getTaskDerivatives() const
{
    return Eigen::VectorXd::Identity(1, 1) * this->derivative_value_;
}

template <typename T_PARAMS, typename PRIO>
void JointSingularityAvoidance<T_PARAMS, PRIO>::calculate()
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    const int32_t joint_idx = this->constraint_params_.joint_idx_;
    std::vector<double> limits_min = limiter_params.limits_min;
    std::vector<double> limits_max = limiter_params.limits_max;
    const double limit_min = limiter_params.limits_min[joint_idx];
    const double limit_max = limiter_params.limits_max[joint_idx];
    const double joint_pos = this->joint_states_.current_q_(joint_idx);

    this->abs_delta_max_ = std::abs(limit_max - joint_pos);
    this->rel_max_ = std::abs(this->abs_delta_max_ / limit_max);

    this->abs_delta_min_ = std::abs(joint_pos - limit_min);
    this->rel_min_ = std::abs(this->abs_delta_min_ / limit_min);

    const double rel_val = this->rel_max_ < this->rel_min_ ? this->rel_max_ : this->rel_min_;

    this->calcValue();
    this->calcPartialValues();
    this->calcDerivativeValue();
    this->state_.setState(DANGER);  // always active task

    // Compute prediction
    const double pred_delta_max = std::abs(limit_max - this->jnts_prediction_.q(joint_idx));
    const double pred_rel_max = std::abs(pred_delta_max / limit_max);
    const double pred_delta_min = std::abs(this->jnts_prediction_.q(joint_idx) - limit_min);
    const double pred_rel_min = std::abs(pred_delta_min / limit_min);
    const double pred_rel_val = pred_rel_max < pred_rel_min ? pred_rel_max : pred_rel_min;

//    if (this->state_.getCurrent() == CRITICAL && pred_rel_val < rel_val)
//    {
//        ROS_WARN_STREAM(this->getTaskId() << ": Current JSA state is CRITICAL but prediction is smaller than current rel_val -> Stay in CRIT.");
//    }
//    else if (rel_val < 0.1 || pred_rel_val < 0.1)
//    {
//        this->state_.setState(CRITICAL);  // always highest task -> avoid HW destruction.
//    }
//    else
//    {
//        this->state_.setState(DANGER);  // always active task -> avoid HW destruction.
//    }

}

template <typename T_PARAMS, typename PRIO>
double JointSingularityAvoidance<T_PARAMS, PRIO>::getActivationGain() const
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    const int32_t joint_idx = this->constraint_params_.joint_idx_;
    const double joint_pos = this->joint_states_.current_q_(joint_idx);

    double activation_gain = 1.0;
    return activation_gain;
}

/// Returns a value for k_H to weight the partial values for e.g. GPM
template <typename T_PARAMS, typename PRIO>
double JointSingularityAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    double k_H = params.k_H_jsa;
    return k_H;
}

template <typename T_PARAMS, typename PRIO>
ConstraintTypes JointSingularityAvoidance<T_PARAMS, PRIO>::getType() const
{
    return JSA;
}

/// Calculate values of the JSA cost function.
template <typename T_PARAMS, typename PRIO>
void JointSingularityAvoidance<T_PARAMS, PRIO>::calcValue()
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    const int32_t joint_idx = this->constraint_params_.joint_idx_;
    std::vector<double> limits_min = limiter_params.limits_min;
    std::vector<double> limits_max = limiter_params.limits_max;
    const double joint_pos = this->joint_states_.current_q_(joint_idx);

//    double nom = (joint_pos - (limits_max[joint_idx] + limits_min[joint_idx])/2);
//    double denom = (limits_max[joint_idx] - limits_min[joint_idx]);
    Eigen::MatrixXd jac = this->jacobian_data_;
    Eigen::MatrixXd prod = jac * jac.transpose();
    double d = prod.determinant();
    double mom = 100*std::sqrt(std::abs(d));

    this->last_value_ = this->value_;
    this->value_ = mom;
    ROS_WARN_STREAM("manipulability: " << mom);

//    this->last_value_ = this->value_;
//    this->value_ = std::abs(w) > ZERO_THRESHOLD ? (1/w) : 1 / DIV0_SAFE;
}

/// Calculate derivative of values.
template <typename T_PARAMS, typename PRIO>
void JointSingularityAvoidance<T_PARAMS, PRIO>::calcDerivativeValue()
{
    this->derivative_value_ = -0.1 * this->value_;
}

/// Calculates values of the gradient of the cost function
template <typename T_PARAMS, typename PRIO>
void JointSingularityAvoidance<T_PARAMS, PRIO>::calcPartialValues()
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    const double joint_pos = this->joint_states_.current_q_(this->constraint_params_.joint_idx_);
    const double joint_vel = this->joint_states_.current_q_dot_(this->constraint_params_.joint_idx_);
    const double limits_min = limiter_params.limits_min[this->constraint_params_.joint_idx_];
    const double limits_max = limiter_params.limits_max[this->constraint_params_.joint_idx_];
    const int32_t joint_idx = this->constraint_params_.joint_idx_;

    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());


    PInvBySVD pinv_calc;
    KDL::ChainJntToJacDotSolver J_dot_solver(params.chain);

    KDL::Jacobian J_dot(params.chain.getNrOfJoints());

    // JointVelocity predictions
    KDL::FrameVel frame_vel;

    KDL::JntArrayVel jnts_prediction_chain(params.dof);

    for (unsigned int i = 0; i < params.dof; i++)
    {
        jnts_prediction_chain.q(i) = this->jnts_prediction_.q(i);
        jnts_prediction_chain.qdot(i) = this->jnts_prediction_.qdot(i);
    }
    // Calculate prediction for the mainipulator
    int error = this->fk_solver_vel_.JntToCart(jnts_prediction_chain, frame_vel, joint_idx);
    if (error != 0)
    {
        ROS_ERROR_STREAM("Could not calculate twist for frame: " << joint_idx  << ". Error Code: " << error << " (" << this->fk_solver_vel_.strError(error) << ")");
        return;
    }

    J_dot_solver.JntToJacDot(jnts_prediction_chain,J_dot, -1);

    Eigen::MatrixXd jac = this->jacobian_data_;

    Eigen::MatrixXd prod = jac * jac.transpose();
    double d = prod.determinant();
    double mom = std::sqrt(std::abs(d));
    double dmom_dq = 0;




    Eigen::MatrixXd dJ_dq_J_inverse = J_dot.data * pinv_calc.calculate(jac);
    dmom_dq = -100*mom * dJ_dq_J_inverse.trace();

    ROS_INFO_STREAM("dmom_dq: " << dmom_dq);

//    Eigen::JacobiSVD<Eigen::MatrixXd> svd(dJ_dq_J_inverse, Eigen::ComputeThinU | Eigen::ComputeThinV);


    partial_values(this->constraint_params_.joint_idx_) = dmom_dq;
    this->partial_values_ = partial_values;
    this->jacobian_data_old_ = jac;
    this->init_ = true;
}
/* END JointSingularityAvoidance **************************************************************************************/

#endif  // COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_JSA_IMPL_H
