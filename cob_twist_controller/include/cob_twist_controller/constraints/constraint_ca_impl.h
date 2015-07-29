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

#include "cob_twist_controller/constraints/constraint.h"
#include "cob_twist_controller/constraints/constraint_params.h"

#include "cob_twist_controller/damping_methods/damping.h"
#include "cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h"

#include <eigen_conversions/eigen_kdl.h>

/* BEGIN CollisionAvoidance *************************************************************************************/
template <typename T_PARAMS, typename PRIO>
std::string CollisionAvoidance<T_PARAMS, PRIO>::getTaskId() const
{
    std::ostringstream oss;
    oss << this->member_inst_cnt_;
    oss << "_";
    oss << this->priority_;
    std::string taskid = "CollisionAvoidance_" + oss.str();
    return taskid;
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getCriticalValue() const
{
    ObstacleDistanceInfo d = this->constraint_params_.current_distance_;
    return d.min_distance;
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getActivationGain() const
{
    double activation_gain;
    double activation = this->getActivationThreshold();
    ObstacleDistanceInfo d = this->constraint_params_.current_distance_;
    double activation_buffer_region = this->getActivationThresholdWithBuffer();

    if (d.min_distance < activation) // activation == d_m
    {
        activation_gain = 1.0;
    }
    else if(d.min_distance < activation_buffer_region) // activation_buffer_region == d_i
    {
        activation_gain = 0.5 * (1.0 - cos(M_PI * (d.min_distance - activation) / (activation_buffer_region - activation)));
    }
    else
    {
        activation_gain = 0.0;
    }

    return activation_gain;
}

template <typename T_PARAMS, typename PRIO>
void CollisionAvoidance<T_PARAMS, PRIO>::calculate()
{
    ObstacleDistanceInfo d = this->constraint_params_.current_distance_;

    mvg_avg_distances_.addElement(d.min_distance);

    Eigen::Vector3d distance_vec = d.nearest_point_frame_vector - d.nearest_point_obstacle_vector;
    mvg_avg_dist_vec_.addElement(distance_vec);

    Eigen::Vector3d coll_pnt_vec = d.nearest_point_frame_vector - d.frame_vector;
    mvg_avg_coll_pnt_vec_.addElement(coll_pnt_vec);

    this->calcValue();
    this->calcDerivativeValue();
    this->calcPartialValues();

    const double pred_min_dist = this->predictValue();
    const double activation = this->getActivationThreshold();
    const double critical = 0.5 * activation;
    const double activation_buffer = this->getActivationThresholdWithBuffer();

    if(this->state_.getCurrent() == CRITICAL && pred_min_dist < d.min_distance)
    {
        ROS_WARN_STREAM(d.frame_id << "Current state is CRITICAL but prediction is smaller than current dist -> Stay in CRIT.");
    }
    else if(d.min_distance < critical || pred_min_dist < critical)
    {
        if(pred_min_dist < critical)
        {
            ROS_WARN_STREAM(d.frame_id << "pred_min_dist < critical!!!");
        }

        this->state_.setState(CRITICAL);
    }
    else if(d.min_distance < activation_buffer)
    {
        this->state_.setState(DANGER);
    }
    else
    {
        this->state_.setState(NORMAL);
    }
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getActivationThresholdWithBuffer() const
{
    return this->getActivationThreshold() * (1.0 + ACTIVATION_BUFFER);
}


template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::calcValue()
{
    ObstacleDistanceInfo d = this->constraint_params_.current_distance_;
    this->last_value_ = this->value_;
    // this->value_ = pow(d.min_distance - this->getActivationThreshold(), 2.0);
    double avg = 0.0;
    mvg_avg_distances_.calcWeightedMovingAverage(avg);
    this->value_ = pow(avg - this->getActivationThresholdWithBuffer(), 2.0);
    return this->value_;
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::predictValue()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    const ObstacleDistanceInfo d = this->constraint_params_.current_distance_;
    this->prediction_value_ = std::numeric_limits<double>::max();

    std::vector<std::string>::const_iterator str_it = std::find(params.frame_names.begin(),
                                                                params.frame_names.end(),
                                                                d.frame_id);
    if (params.frame_names.end() != str_it)
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

        Eigen::Vector3d delta_pred_vel = pred_twist_vel + pred_twist_rot.cross(d.nearest_point_frame_vector);
        double cycle = (ros::Time::now() - this->last_pred_time_).toSec();
        this->last_pred_time_ = ros::Time::now();
        Eigen::Vector3d pred_pos = d.nearest_point_frame_vector + delta_pred_vel * cycle;
        this->prediction_value_ = (d.nearest_point_obstacle_vector - pred_pos).norm();

        ROS_ERROR_STREAM("Minimal distance for " << d.frame_id << ": " << d.min_distance);
        ROS_ERROR_STREAM("Predicted distance for " << d.frame_id << ": " << this->prediction_value_);
    }

    return this->prediction_value_;
}





template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::calcDerivativeValue()
{
    this->derivative_value_ = -0.1 * this->value_; // exponential decay
    return this->derivative_value_;
}

template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd CollisionAvoidance<T_PARAMS, PRIO>::calcPartialValues()
{
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    const TwistControllerParams& params = this->constraint_params_.getParams();
    int size_of_frames = params.frame_names.size();
    ObstacleDistanceInfo d = this->constraint_params_.current_distance_;
    if (this->getActivationThresholdWithBuffer() > d.min_distance)
    {
        std::vector<std::string>::const_iterator str_it = std::find(params.frame_names.begin(),
                                                                    params.frame_names.end(),
                                                                    d.frame_id);
        if (params.frame_names.end() != str_it)
        {
            // Eigen::Vector3d collision_pnt_vector = d.nearest_point_frame_vector - d.frame_vector;
            // Eigen::Vector3d distance_vec = d.nearest_point_frame_vector - d.nearest_point_obstacle_vector;

            Eigen::Vector3d collision_pnt_vector = Eigen::Vector3d::Zero();
            mvg_avg_coll_pnt_vec_.calcWeightedMovingAverage(collision_pnt_vector);

            Eigen::Vector3d distance_vec = Eigen::Vector3d::Zero();
            mvg_avg_dist_vec_.calcWeightedMovingAverage(distance_vec);

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
                return partial_values;
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
            const double denom = d.min_distance > 0.0 ? d.min_distance : DIV0_SAFE;
            partial_values =  (2.0 * ((d.min_distance - this->getActivationThresholdWithBuffer()) / denom) * term_2nd);
        }
        else
        {
            ROS_ERROR_STREAM("Frame id not found: " << d.frame_id);
        }
    }

    this->partial_values_ = partial_values;
    return this->partial_values_;
}


/// Returns the threshold of the cost function to become active.
template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getActivationThreshold() const
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    return params.activation_threshold_ca; // in [m]
}


/// Returns a value for k_H to weight the partial values for GPM e.g.
template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    double magnitude = 0.0;
    double activation = this->getActivationThreshold();
    ObstacleDistanceInfo d = this->constraint_params_.current_distance_;

    if (d.min_distance < activation && d.min_distance > 0.0)
    {
        magnitude = pow(activation / d.min_distance, 2.0) - 1.0;
    }

    double k_H = params.k_H_ca;
    return k_H * magnitude;
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
    return this->partial_values_.transpose();
}

/**
 * 1x1 Vector returning the task derivative of a CA constraint.
 * One row task Jacobian <-> One dim derivative.
 * @return The derivative value.
 */
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd CollisionAvoidance<T_PARAMS, PRIO>::getTaskDerivatives() const
{
    return Eigen::VectorXd::Identity(1, 1) * this->derivative_value_;
}

template <typename T_PARAMS, typename PRIO>
Task_t CollisionAvoidance<T_PARAMS, PRIO>::createTask()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
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
