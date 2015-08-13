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

#ifndef CONSTRAINT_CA_IMPL_H_
#define CONSTRAINT_CA_IMPL_H_

#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <ros/ros.h>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>

#include <eigen_conversions/eigen_kdl.h>

#include "cob_twist_controller/constraints/constraint.h"
#include "cob_twist_controller/constraints/constraint_params.h"

#include "cob_twist_controller/damping_methods/damping.h"
#include "cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h"

/* BEGIN CollisionAvoidance *************************************************************************************/
template <typename T_PARAMS, typename PRIO>
std::string CollisionAvoidance<T_PARAMS, PRIO>::getTaskId() const
{
    const std::string frame_id = this->constraint_params_.id_;
    std::ostringstream oss;
    oss << this->member_inst_cnt_;
    oss << "_";
    oss << frame_id;
    oss << "_";
    oss << this->priority_;
    std::string taskid = "CollisionAvoidance_" + oss.str();
    return taskid;
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getCriticalValue() const
{
    double min_distance = std::numeric_limits<double>::max();
    for(std::vector<ObstacleDistanceData>::const_iterator it = this->constraint_params_.current_distances_.begin();
            it != this->constraint_params_.current_distances_.end();
            ++it)
    {
        if(it->min_distance < min_distance)
        {
            min_distance = it->min_distance;
        }
    }

    return min_distance;
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getActivationGain(double current_cost_func_value) const
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    double activation_gain;
    const double activation = params.thresholds_ca.activation;
    const double activation_buffer_region = params.thresholds_ca.activation_with_buffer;

    if (current_cost_func_value < activation) // activation == d_m
    {
        activation_gain = 1.0;
    }
    else if(current_cost_func_value < activation_buffer_region) // activation_buffer_region == d_i
    {
        activation_gain = 0.5 * (1.0 - cos(M_PI * (current_cost_func_value - activation) / (activation_buffer_region - activation)));
    }
    else
    {
        activation_gain = 0.0;
    }

    return activation_gain;
}


template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getActivationGain() const
{
    return 1.0;
}

template <typename T_PARAMS, typename PRIO>
void CollisionAvoidance<T_PARAMS, PRIO>::calculate()
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;

    this->calcValue();
    this->calcDerivativeValue();
    this->calcPartialValues();

    const double pred_min_dist = this->predictValue();
    const double activation = params.thresholds_ca.activation;
    const double critical = params.thresholds_ca.critical;
    const double activation_buffer = params.thresholds_ca.activation_with_buffer;
    const double crit_min_distance = this->getCriticalValue();

    if(this->state_.getCurrent() == CRITICAL && pred_min_dist < crit_min_distance)
    {
        ROS_WARN_STREAM(this->getTaskId() << ": Current state is CRITICAL but prediction " << pred_min_dist << " is smaller than current dist " << crit_min_distance << " -> Stay in CRIT.");
    }
    else if(crit_min_distance < critical || pred_min_dist < critical)
    {
        if(pred_min_dist < critical)
        {
            ROS_WARN_STREAM(this->getTaskId() << ": pred_min_dist < critical!!!");
        }

        this->state_.setState(CRITICAL);
    }
    else if(crit_min_distance < activation_buffer)
    {
        this->state_.setState(DANGER);
    }
    else
    {
        this->state_.setState(NORMAL);
    }
}


template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::calcValue()
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    std::vector<double> relevant_values;
    for(std::vector<ObstacleDistanceData>::const_iterator it = this->constraint_params_.current_distances_.begin();
            it != this->constraint_params_.current_distances_.end();
            ++it)
    {
        if (params.thresholds_ca.activation_with_buffer > it->min_distance)
        {
            const double activation_gain = this->getActivationGain(it->min_distance);
            const double magnitude = std::abs(this->getSelfMotionMagnitude(it->min_distance)); // important only for task!
            double value = activation_gain * magnitude * pow(it->min_distance - params.thresholds_ca.activation_with_buffer, 2.0);
            relevant_values.push_back(value);
        }
    }

    if(relevant_values.size() > 0)
    {
        this->values_ = Eigen::VectorXd::Zero(relevant_values.size());
    }

    for(uint32_t idx = 0; idx < relevant_values.size(); ++idx)
    {
        this->values_(idx) = relevant_values.at(idx);
    }

    return this->value_;
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::predictValue()
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    this->prediction_value_ = std::numeric_limits<double>::max();

    double cycle = (ros::Time::now() - this->last_pred_time_).toSec();

    std::vector<std::string>::const_iterator str_it = std::find(params.frame_names.begin(),
                                                                params.frame_names.end(),
                                                                this->constraint_params_.id_);
    if (params.frame_names.end() != str_it)
    {
        if(this->constraint_params_.current_distances_.size() > 0)
        {
            uint32_t frame_number = (str_it - params.frame_names.begin()) + 1; // segment nr not index represents frame number
            KDL::FrameVel frame_vel;

            // Calculate prediction for pos and vel
            if(0 != this->fk_solver_vel_.JntToCart(this->jnts_prediction_, frame_vel, frame_number))
            {
                ROS_ERROR_STREAM("Could not calculate twist for frame: " << frame_number);
                return std::numeric_limits<double>::max();
            }

            KDL::Twist twist = frame_vel.GetTwist(); // predicted frame twist

            Eigen::Vector3d pred_twist_vel;
            tf::vectorKDLToEigen(twist.vel, pred_twist_vel);

            Eigen::Vector3d pred_twist_rot;
            tf::vectorKDLToEigen(twist.rot, pred_twist_rot);

            std::vector<ObstacleDistanceData>::const_iterator it = this->constraint_params_.current_distances_.begin();
            ObstacleDistanceData critical_data = *it;
            for( ; it != this->constraint_params_.current_distances_.end();
                   ++it)
            {
                if(it->min_distance < critical_data.min_distance)
                {
                    critical_data = *it;
                }
            }

            Eigen::Vector3d delta_pred_vel = pred_twist_vel + pred_twist_rot.cross(critical_data.nearest_point_frame_vector);
            Eigen::Vector3d pred_pos = critical_data.nearest_point_frame_vector + delta_pred_vel * cycle;
            this->prediction_value_ = (critical_data.nearest_point_obstacle_vector - pred_pos).norm();
        }
    }
    else
    {
        ROS_ERROR_STREAM("Frame ID not found: " << this->constraint_params_.id_);
    }

    this->last_pred_time_ = ros::Time::now();
    return this->prediction_value_;
}


template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::calcDerivativeValue()
{
    this->derivative_value_ = -0.2 * this->value_; // exponential decay experimentally chosen -0.1
    this->derivative_values_ = -0.2 * this->values_;
    return this->derivative_value_;
}



/**
 * Calculate the partial values for each obstacle with the current link.
 * For GPM the partial values are summed.
 * For task constraint the task Jacobian is created: Each row is the partial value vector of one collision pair.
 * ATTENTION: The magnitude and activation gain are considered only for GPM here.
 * @return The partial values vector consisting of the sum of all collision pair partial values.
 */
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd CollisionAvoidance<T_PARAMS, PRIO>::calcPartialValues()
{
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    Eigen::VectorXd sum_partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    int size_of_frames = params.frame_names.size();
    std::vector<Eigen::VectorXd> vec_partial_values;

    std::vector<std::string>::const_iterator str_it = std::find(params.frame_names.begin(),
                                                                params.frame_names.end(),
                                                                this->constraint_params_.id_);

    for(std::vector<ObstacleDistanceData>::const_iterator it = this->constraint_params_.current_distances_.begin();
            it != this->constraint_params_.current_distances_.end();
            ++it)
    {
        if (params.thresholds_ca.activation_with_buffer > it->min_distance)
        {
            if (params.frame_names.end() != str_it)
            {
                Eigen::Vector3d collision_pnt_vector = it->nearest_point_frame_vector - it->frame_vector;
                Eigen::Vector3d distance_vec = it->nearest_point_frame_vector - it->nearest_point_obstacle_vector;

                /*
                 * Create a skew-symm matrix as transformation between the segment root and the critical point.
                 */
                Eigen::Matrix3d skew_symm;
                skew_symm <<    0.0,                        collision_pnt_vector.z(), -collision_pnt_vector.y(),
                                -collision_pnt_vector.z(),  0.0,                       collision_pnt_vector.x(),
                                 collision_pnt_vector.y(), -collision_pnt_vector.x(),  0.0;

                Eigen::Matrix3d ident = Eigen::Matrix3d::Identity();
                Eigen::Matrix<double,6,6> T;
                T.block(0, 0, 3, 3) << ident;
                T.block(0, 3, 3, 3) << skew_symm;
                T.block(3, 0, 3, 3) << Eigen::Matrix3d::Zero();
                T.block(3, 3, 3, 3) << ident;
                // ****************************************************************************************************

                uint32_t idx = str_it - params.frame_names.begin();
                uint32_t frame_number = idx + 1; // segment nr not index represents frame number

                KDL::Jacobian new_jac_chain(size_of_frames);

                KDL::JntArray ja = this->joint_states_.current_q_;
                if(0 != this->jnt_to_jac_.JntToJac(ja, new_jac_chain, frame_number))
                {
                    ROS_ERROR_STREAM("Failed to calculate JntToJac.");
                    return sum_partial_values;
                }

                Matrix6Xd_t jac_extension = this->jacobian_data_;
                jac_extension.block(0, 0, new_jac_chain.data.rows(), new_jac_chain.data.cols()) = new_jac_chain.data;
                Matrix6Xd_t crit_pnt_jac = T * jac_extension;
                Eigen::Matrix3Xd m_transl = Eigen::Matrix3Xd::Zero(3, crit_pnt_jac.cols());
                m_transl << crit_pnt_jac.row(0),
                        crit_pnt_jac.row(1),
                        crit_pnt_jac.row(2);

                double vec_norm = distance_vec.norm();
                vec_norm = vec_norm > 0.0 ? vec_norm : DIV0_SAFE;
                Eigen::VectorXd term_2nd = (m_transl.transpose()) * (distance_vec / vec_norm); // use the unit vector only for direction!

                // Gradient of the cost function from: Strasse O., Escande A., Mansard N. et al.
                // "Real-Time (Self)-Collision Avoidance Task on a HRP-2 Humanoid Robot", 2008 IEEE International Conference
                const double denom = it->min_distance > 0.0 ? it->min_distance : DIV0_SAFE;
                const double activation_gain = this->getActivationGain(it->min_distance);
                const double magnitude = this->getSelfMotionMagnitude(it->min_distance);
                partial_values = (2.0 * ((it->min_distance - params.thresholds_ca.activation_with_buffer) / denom) * term_2nd);
                // only consider the gain for the partial values, because of GPM, not for the task jacobian!
                sum_partial_values += (activation_gain * magnitude * partial_values);
                vec_partial_values.push_back(partial_values);
            }
            else
            {
                ROS_ERROR_STREAM("Frame id not found: " << this->constraint_params_.id_);
            }
        }
    }

    if(vec_partial_values.size() > 0)
    {
        this->task_jacobian_.resize(vec_partial_values.size(), this->jacobian_data_.cols());
    }


    for(uint32_t idx = 0; idx < vec_partial_values.size(); ++idx)
    {
        this->task_jacobian_.block(idx, 0, 1, this->jacobian_data_.cols()) = vec_partial_values.at(idx).transpose();
    }

    this->partial_values_.resize(sum_partial_values.rows(), 1);
    this->partial_values_ = sum_partial_values;
    return this->partial_values_;
}


/// Returns a value for magnitude
template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(double current_distance_value) const
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    const double activation = params.thresholds_ca.activation_with_buffer;
    double magnitude = 0.0;

    if (current_distance_value < activation)
    {
        if(current_distance_value > 0.0)
        {
            magnitude = pow(activation / current_distance_value, 2.0) - 1.0;
        }
        else
        {
            magnitude = activation / 0.000001; // strong magnitude
        }
    }

    double k_H = params.k_H_ca;
    return k_H * magnitude;
}



/// Returns a value for k_H to weight the partial values for GPM e.g.
template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    return 1.0;
}

template <typename T_PARAMS, typename PRIO>
ConstraintTypes CollisionAvoidance<T_PARAMS, PRIO>::getType() const
{
    return CA;
}

/**
 * Critical Point Jacobian: Each critical point is represented by one CA constraint.
 * So the partial values represent a one row task Jacobian.
 * @return Partial values as task Jacobian.
 */
template <typename T_PARAMS, typename PRIO>
Eigen::MatrixXd CollisionAvoidance<T_PARAMS, PRIO>::getTaskJacobian() const
{
    return this->task_jacobian_;
}

/**
 * 1x1 Vector returning the task derivative of a CA constraint.
 * One row task Jacobian <-> One dim derivative.
 * @return The derivative value.
 */
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd CollisionAvoidance<T_PARAMS, PRIO>::getTaskDerivatives() const
{
    //return Eigen::VectorXd::Identity(1, 1) * this->derivative_value_;
    return this->derivative_values_;
}

template <typename T_PARAMS, typename PRIO>
Task_t CollisionAvoidance<T_PARAMS, PRIO>::createTask()
{
    const TwistControllerParams& params = this->constraint_params_.tc_params_;
    TwistControllerParams adapted_params;
    adapted_params.damping_method = CONSTANT;
    adapted_params.damping_factor = params.damping_ca;
    adapted_params.eps_truncation = 0.0;

    Task_t task(this->getPriority(),
                this->getTaskId(),
                this->getTaskJacobian(),
                this->getTaskDerivatives(),
                this->getType());

    task.tcp_ = adapted_params;
    task.db_.reset(DampingBuilder::createDamping(adapted_params));

    return task;
}

/* END CollisionAvoidance ***************************************************************************************/

#endif /* CONSTRAINT_CA_IMPL_H_ */
