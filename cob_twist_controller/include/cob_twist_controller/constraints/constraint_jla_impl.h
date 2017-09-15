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


#ifndef COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_JLA_IMPL_H
#define COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_JLA_IMPL_H

#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>

#include "cob_twist_controller/constraints/constraint.h"
#include "cob_twist_controller/constraints/constraint_params.h"

#include "cob_twist_controller/damping_methods/damping.h"
#include "cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h"

#include <eigen_conversions/eigen_kdl.h>

/* BEGIN JointLimitAvoidance ************************************************************************************/
template <typename T_PARAMS, typename PRIO>
std::string JointLimitAvoidance<T_PARAMS, PRIO>::getTaskId() const
{
    std::ostringstream oss;
    oss << this->member_inst_cnt_;
    oss << "_Joint#";
    oss << this->constraint_params_.joint_idx_;
    oss << "_";
    oss << this->priority_;
    std::string taskid = "JointLimitAvoidance_" + oss.str();
    return taskid;
}

template <typename T_PARAMS, typename PRIO>
Eigen::MatrixXd JointLimitAvoidance<T_PARAMS, PRIO>::getTaskJacobian() const
{
    return this->partial_values_.transpose();
}

template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidance<T_PARAMS, PRIO>::getTaskDerivatives() const
{
    return Eigen::VectorXd::Identity(1, 1) * this->derivative_value_;
}

template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidance<T_PARAMS, PRIO>::calculate()
{
    const ConstraintParams& params = this->constraint_params_.params_;
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    const int32_t joint_idx = this->constraint_params_.joint_idx_;
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

    // Compute prediction
    const double pred_delta_max = std::abs(limit_max - this->jnts_prediction_.q(joint_idx));
    const double pred_rel_max = std::abs(pred_delta_max / limit_max);
    const double pred_delta_min = std::abs(this->jnts_prediction_.q(joint_idx) - limit_min);
    const double pred_rel_min = std::abs(pred_delta_min / limit_min);
    const double pred_rel_val = pred_rel_max < pred_rel_min ? pred_rel_max : pred_rel_min;

    const double activation = params.thresholds.activation;
    const double critical = params.thresholds.critical;

    if (this->state_.getCurrent() == CRITICAL && pred_rel_val < rel_val)
    {
        ROS_WARN_STREAM(this->getTaskId() << ": Current state is CRITICAL but prediction is smaller than current rel_val -> Stay in CRIT.");
    }
    else if (rel_val < critical || pred_rel_val < critical)
    {
        this->state_.setState(CRITICAL);  // always highest task -> avoid HW destruction.
    }
    else
    {
        this->state_.setState(DANGER);  // always active task -> avoid HW destruction.
    }
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getActivationGain() const
{
    const ConstraintParams& params = this->constraint_params_.params_;
    const double activation_threshold = params.thresholds.activation;
    const double activation_buffer_region = params.thresholds.activation_with_buffer;  // [%]

    double activation_gain;
    const double rel_delta = this->rel_min_ < this->rel_max_ ? this->rel_min_ : this->rel_max_;

    if (rel_delta < activation_threshold)
    {
        activation_gain = 1.0;
    }
    else if (rel_delta < activation_buffer_region)
    {
        activation_gain = 0.5 * (1.0 + cos(M_PI * (rel_delta - activation_threshold) / (activation_buffer_region - activation_threshold)));
    }
    else
    {
        activation_gain = 0.0;
    }

    if (activation_gain < 0.0)
    {
        activation_gain = 0.0;
    }

    return activation_gain;
}

/// Returns a value for k_H to weight the partial values for e.g. GPM
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    return this->constraint_params_.params_.k_H;
}

/// Calculate values of the JLA cost function.
template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidance<T_PARAMS, PRIO>::calcValue()
{
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    const int32_t joint_idx = this->constraint_params_.joint_idx_;
    std::vector<double> limits_min = limiter_params.limits_min;
    std::vector<double> limits_max = limiter_params.limits_max;
    const double joint_pos = this->joint_states_.current_q_(joint_idx);

    double nom = pow(limits_max[joint_idx] - limits_min[joint_idx], 2.0);
    double denom = (limits_max[joint_idx] - joint_pos) * (joint_pos - limits_min[joint_idx]);

    this->last_value_ = this->value_;
    this->value_ = std::abs(denom) > ZERO_THRESHOLD ? nom / denom : nom / DIV0_SAFE;
}

/// Calculate derivative of values.
template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidance<T_PARAMS, PRIO>::calcDerivativeValue()
{
    this->derivative_value_ = -0.1 * this->value_;
}

/// Calculates values of the gradient of the cost function
template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidance<T_PARAMS, PRIO>::calcPartialValues()
{
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    const double joint_pos = this->joint_states_.current_q_(this->constraint_params_.joint_idx_);
    const double joint_vel = this->joint_states_.current_q_dot_(this->constraint_params_.joint_idx_);
    const double limits_min = limiter_params.limits_min[this->constraint_params_.joint_idx_];
    const double limits_max = limiter_params.limits_max[this->constraint_params_.joint_idx_];
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());

    const double min_delta = (joint_pos - limits_min);
    const double max_delta = (limits_max - joint_pos);
    const double nominator = (2.0 * joint_pos - limits_min - limits_max) * (limits_max - limits_min) * (limits_max - limits_min);
    const double denom = 4.0 * min_delta * min_delta * max_delta * max_delta;

    partial_values(this->constraint_params_.joint_idx_) = std::abs(denom) > ZERO_THRESHOLD ? nominator / denom : nominator / DIV0_SAFE;
    this->partial_values_ = partial_values;
}
/* END JointLimitAvoidance **************************************************************************************/

/* BEGIN JointLimitAvoidanceMid ************************************************************************************/
template <typename T_PARAMS, typename PRIO>
std::string JointLimitAvoidanceMid<T_PARAMS, PRIO>::getTaskId() const
{
    std::ostringstream oss;
    oss << this->member_inst_cnt_;
    oss << "_";
    oss << this->priority_;
    std::string taskid = "JointLimitAvoidanceMid_" + oss.str();
    return taskid;
}

template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidanceMid<T_PARAMS, PRIO>::calculate()
{
    this->calcValue();
    this->calcDerivativeValue();
    this->calcPartialValues();
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getActivationGain() const
{
    return 1.0;
}

/// Returns a value for k_H to weight the partial values for e.g. GPM
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    return this->constraint_params_.params_.k_H;
}

/// Calculate values of the JLA cost function.
template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidanceMid<T_PARAMS, PRIO>::calcValue()
{
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    std::vector<double> limits_min = limiter_params.limits_min;
    std::vector<double> limits_max = limiter_params.limits_max;
    const KDL::JntArray joint_pos = this->joint_states_.current_q_;
    double H_q = 0.0;
    for (uint8_t i = 0; i < joint_pos.rows() ; ++i)
    {
        double jnt_pos_with_step = joint_pos(i);
        double nom = pow(limits_max[i] - limits_min[i], 2.0);
        double denom = (limits_max[i] - jnt_pos_with_step) * (jnt_pos_with_step - limits_min[i]);
        H_q += nom / denom;
    }

    this->value_ = H_q / 4.0;
}

/// Calculate derivative of values.
template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidanceMid<T_PARAMS, PRIO>::calcDerivativeValue()
{
    ros::Time now = ros::Time::now();
    double cycle = (now - this->last_time_).toSec();
    this->last_time_ = now;

    if (cycle > 0.0)
    {
        this->derivative_value_ = (this->value_ - this->last_value_) / cycle;
    }
    else
    {
        this->derivative_value_ = (this->value_ - this->last_value_) / DEFAULT_CYCLE;
    }
}

/// Calculates values of the gradient of the cost function
template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidanceMid<T_PARAMS, PRIO>::calcPartialValues()
{
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    const KDL::JntArray joint_pos = this->joint_states_.current_q_;
    std::vector<double> limits_min = limiter_params.limits_min;
    std::vector<double> limits_max = limiter_params.limits_max;

    uint8_t vec_rows = static_cast<uint8_t>(joint_pos.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());

    for (uint8_t i = 0; i < vec_rows; ++i)  // max limiting the arm joints but not the extensions.
    {
        double min_delta = joint_pos(i) - limits_min[i];
        double max_delta = limits_max[i] - joint_pos(i);
        if (min_delta * max_delta < 0.0)
        {
            ROS_WARN_STREAM("Limit of joint " << int(i) << " reached: " << std::endl
                            << "pos=" << joint_pos(i) << ";lim_min=" << limits_min[i] << ";lim_max=" << limits_max[i]);
        }

        // Liegeois method can also be found in Chan paper ISSN 1042-296X [Page 288]
        double limits_mid = 1.0 / 2.0 * (limits_max[i] + limits_min[i]);
        double nominator = joint_pos(i) - limits_mid;
        double denom = pow(limits_max[i] - limits_min[i], 2.0);
        partial_values(i) = nominator / denom;
    }

    this->partial_values_ = partial_values;
}
/* END JointLimitAvoidanceMid ************************************************************************************/

/* BEGIN JointLimitAvoidanceIneq ************************************************************************************/
template <typename T_PARAMS, typename PRIO>
std::string JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getTaskId() const
{
    std::ostringstream oss;
    oss << this->member_inst_cnt_;
    oss << "_";
    oss << this->priority_;
    std::string taskid = "JointLimitAvoidanceIneq_" + oss.str();
    return taskid;
}

template <typename T_PARAMS, typename PRIO>
Eigen::MatrixXd JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getTaskJacobian() const
{
    return this->partial_values_.transpose();
}

template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getTaskDerivatives() const
{
    return Eigen::VectorXd::Identity(1, 1) * this->derivative_value_;
}

template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidanceIneq<T_PARAMS, PRIO>::calculate()
{
    const ConstraintParams& params = this->constraint_params_.params_;
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    const int32_t joint_idx = this->constraint_params_.joint_idx_;
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

    // Compute prediction
    const double pred_delta_max = std::abs(limit_max - this->jnts_prediction_.q(joint_idx));
    const double pred_rel_max = std::abs(pred_delta_max / limit_max);

    const double pred_delta_min = std::abs(this->jnts_prediction_.q(joint_idx) - limit_min);
    const double pred_rel_min = std::abs(pred_delta_min / limit_min);

    this->prediction_value_ = pred_rel_max < pred_rel_min ? pred_rel_max : pred_rel_min;

    double activation = params.thresholds.activation;
    double critical = params.thresholds.critical;

    if (this->state_.getCurrent() == CRITICAL && this->prediction_value_ < rel_val)
    {
        ROS_WARN_STREAM(this->getTaskId() << ": Current state is CRITICAL but prediction is smaller than current rel_val -> Stay in CRIT.");
    }
    else if (rel_val < critical || this->prediction_value_ < critical)
    {
        if (this->prediction_value_ < critical)
        {
            ROS_WARN_STREAM(this->getTaskId() << ": pred_val < critical!!!");
        }

        this->state_.setState(CRITICAL);
    }
    else
    {
        this->state_.setState(DANGER);  // GPM always active!
    }
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getActivationGain() const
{
    const ConstraintParams& params = this->constraint_params_.params_;
    const double activation_threshold = params.thresholds.activation;  // [%]
    const double activation_buffer_region = params.thresholds.activation_with_buffer;  // [%]
    double activation_gain;
    double rel_delta;

    if (this->abs_delta_max_ > this->abs_delta_min_)
    {
        rel_delta = this->rel_min_;
    }
    else
    {
        // nearer to max limit
        rel_delta = this->rel_max_;
    }

    if (rel_delta < activation_threshold)
    {
        activation_gain = 1.0;
    }
    else if (rel_delta < activation_buffer_region)
    {
        activation_gain = 0.5 * (1.0 + cos(M_PI * (rel_delta - activation_threshold) / (activation_buffer_region - activation_threshold)));
    }
    else
    {
        activation_gain = 0.0;
    }

    if (activation_gain < 0.0)
    {
        activation_gain = 0.0;
    }

    return activation_gain;
}

/// Returns a value for k_H to weight the partial values for e.g. GPM
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    double factor;
    const ConstraintParams& params = this->constraint_params_.params_;
    if (this->abs_delta_max_ > this->abs_delta_min_ && this->rel_min_ > 0.0)
    {
        factor = (params.thresholds.activation * 1.1 / this->rel_min_) - 1.0;
    }
    else
    {
        if (this->rel_max_ > 0.0)
        {
            factor = (params.thresholds.activation * 1.1 / this->rel_max_) - 1.0;
        }
        else
        {
            factor = 1.0;
        }
    }

    double k_H = factor * params.k_H;
    return k_H;
}

/// Calculate values of the JLA cost function.
template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidanceIneq<T_PARAMS, PRIO>::calcValue()
{
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    int32_t joint_idx = this->constraint_params_.joint_idx_;
    double limit_min = limiter_params.limits_min[joint_idx];
    double limit_max = limiter_params.limits_max[joint_idx];

    double joint_pos = this->joint_states_.current_q_(joint_idx);

    this->last_value_ = this->value_;
    this->value_ = (limit_max - joint_pos) * (joint_pos - limit_min);
}

/// Simple first order differential equation for exponential increase (move away from limit!)
template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidanceIneq<T_PARAMS, PRIO>::calcDerivativeValue()
{
    this->derivative_value_ = 0.1 * this->value_;
}

/// Calculates values of the gradient of the cost function
template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidanceIneq<T_PARAMS, PRIO>::calcPartialValues()
{
    const LimiterParams& limiter_params = this->constraint_params_.limiter_params_;
    int32_t joint_idx = this->constraint_params_.joint_idx_;
    double limit_min = limiter_params.limits_min[joint_idx];
    double limit_max = limiter_params.limits_max[joint_idx];
    double joint_pos = this->joint_states_.current_q_(joint_idx);
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());

    partial_values(this->constraint_params_.joint_idx_) = -2.0 * joint_pos + limit_max + limit_min;
    this->partial_values_ = partial_values;
}
/* END JointLimitAvoidanceIneq **************************************************************************************/

#endif  // COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_JLA_IMPL_H
