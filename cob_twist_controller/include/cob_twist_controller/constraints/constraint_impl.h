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

#ifndef CONSTRAINT_IMPL_H_
#define CONSTRAINT_IMPL_H_

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

/* BEGIN ConstraintsBuilder *************************************************************************************/
/**
 * Static builder method to create damping methods dependent on parameterization.
 */
template <typename PRIO>
std::set<ConstraintBase_t> ConstraintsBuilder<PRIO>::createConstraints(const TwistControllerParams& twist_controller_params,
                                                                      KDL::ChainJntToJacSolver& jnt_to_jac,
                                                                      KDL::ChainFkSolverVel_recursive& fk_solver_vel,
                                                                      CallbackDataMediator& data_mediator)
{
    std::set<ConstraintBase_t> constraints;
    // === Joint limit avoidance part
    if (JLA_ON == twist_controller_params.constraint_jla)
    {
        typedef JointLimitAvoidance<ConstraintParamsJLA, PRIO> Jla_t;
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(twist_controller_params, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<Jla_t > jla(new Jla_t(twist_controller_params.priority_jla, params, data_mediator));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if(JLA_MID_ON == twist_controller_params.constraint_jla)
    {
        // same params as for normal JLA
        typedef JointLimitAvoidanceMid<ConstraintParamsJLA, PRIO> JlaMid_t;
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(twist_controller_params, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<JlaMid_t > jla(new JlaMid_t(twist_controller_params.priority_jla, params, data_mediator));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    if (JLA_INEQ_ON == twist_controller_params.constraint_jla)
    {
        typedef JointLimitAvoidanceIneq<ConstraintParamsJLA, PRIO> Jla_t;
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(twist_controller_params, data_mediator);
        uint32_t startPrio = twist_controller_params.priority_jla;
        for (uint32_t i = 0; i < twist_controller_params.joints.size(); ++i)
        {
            // TODO: take care PRIO could be of different type than UINT32
            params.joint_ = twist_controller_params.joints[i];
            params.joint_idx_ = static_cast<int32_t>(i);
            // copy of params will be created; priority increased with each joint.
            boost::shared_ptr<Jla_t > jla(new Jla_t(startPrio++, params, data_mediator));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
        }
    }
    else
    {
        // JLA_OFF selected.
    }

    // === Collision avoidance part
    if(CA_ON == twist_controller_params.constraint_ca)
    {
        typedef CollisionAvoidance<ConstraintParamsCA, PRIO> CollisionAvoidance_t;
        uint32_t available_dists = data_mediator.obstacleDistancesCnt();
        uint32_t startPrio = twist_controller_params.priority_ca;
        for (uint32_t i = 0; i < available_dists; ++i)
        {
            ConstraintParamsCA params = ConstraintParamFactory<ConstraintParamsCA>::createConstraintParams(twist_controller_params, data_mediator);
            // TODO: take care PRIO could be of different type than UINT32
            boost::shared_ptr<CollisionAvoidance_t > ca(new CollisionAvoidance_t(startPrio--, params, data_mediator, jnt_to_jac, fk_solver_vel));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(ca));
        }
    }
    else
    {
        // CA_OFF selected.
        // Nothing to do here!
        // Create constraints will be called also in case of an unconstraint solver etc.
        // So the log would be filled unnecessarily.
    }

    return constraints;
}
/* END ConstraintsBuilder *******************************************************************************************/

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
    double activation_buffer_region = activation * (1.0 + ACTIVATION_BUFFER);

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
    double pred_min_dist = this->predictValue();

    double activation = this->getActivationThreshold();
    double critical = 0.5 * activation;

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
    else if(d.min_distance < activation)
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
    ObstacleDistanceInfo d = this->constraint_params_.current_distance_;
    this->last_value_ = this->value_;
    // this->value_ = pow(d.min_distance - this->getActivationThreshold(), 2.0);
    double avg = 0.0;
    mvg_avg_distances_.calcWeightedMovingAverage(avg);
    this->value_ = pow(avg - this->getActivationThreshold(), 2.0);
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
    if (this->getActivationThreshold() > d.min_distance)
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
            double denom = d.min_distance > 0.0 ? d.min_distance : DIV0_SAFE;
            partial_values =  (2.0 * ((d.min_distance - this->getActivationThreshold()) / denom) * term_2nd);
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

/* BEGIN JointLimitAvoidance ************************************************************************************/
template <typename T_PARAMS, typename PRIO>
std::string JointLimitAvoidance<T_PARAMS, PRIO>::getTaskId() const
{
    std::ostringstream oss;
    oss << this->member_inst_cnt_;
    oss << "_";
    oss << this->priority_;
    std::string taskid = "JointLimitAvoidance_" + oss.str();
    return taskid;
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getActivationGain() const
{
    return 1.0;
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::predictValue()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    const KDL::JntArray joint_pos = this->jnts_prediction_.q;
    const KDL::JntArray joint_vel = this->jnts_prediction_.qdot;
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;
    uint8_t vec_rows = static_cast<uint8_t>(joint_pos.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    uint8_t used = 0;
    Eigen::VectorXd predict_values = Eigen::VectorXd::Zero(joint_pos.rows(), 1);
    this->pred_active_idx_.clear();
    for(uint8_t i = 0; i < vec_rows; ++i)
    {
        double nom = pow(limits_max[i] - limits_min[i], 2.0);
        double denom = (limits_max[i] - joint_pos(i)) * (joint_pos(i) - limits_min[i]);
        predict_values(i) = nom / (4.0 * denom); // store value of cost function for each joint.

        //See Chan paper ISSN 1042-296X [Page 288]
        if( (joint_vel(i) > 0.0 && ((limits_max[i] - joint_pos(i)) < (joint_pos(i) - limits_min[i])))
                || (joint_vel(i) < 0.0 && ((limits_max[i] - joint_pos(i)) > (joint_pos(i) - limits_min[i]))) )
        {
            double min_delta = (joint_pos(i) - limits_min[i]);
            double max_delta = (limits_max[i] - joint_pos(i));
            double nominator = (2.0 * joint_pos(i) - limits_min[i] - limits_max[i]) * (limits_max[i] - limits_min[i]) * (limits_max[i] - limits_min[i]);
            double denom = 4.0 * min_delta * min_delta * max_delta * max_delta;
            partial_values(used) = nominator / denom;
            ++used;
            this->pred_active_idx_.push_back(i);
        }
    }

    const uint32_t values_rows = predict_values.rows();
    this->predict_derivative_values_.resize(this->pred_active_idx_.size(), 1);
    for(uint8_t i = 0; i < this->pred_active_idx_.size(); ++i)
    {
        uint8_t active_idx = this->pred_active_idx_[i];
        this->predict_derivative_values_(i) = this->exp_decay_ * this->values_(active_idx);
    }

    this->predict_partial_values_ = partial_values;
    this->prediction_value_ = partial_values.sum();
    return this->prediction_value_;
}

template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidance<T_PARAMS, PRIO>::calculate()
{
    this->calcValue();
    this->calcPartialValues();
    this->calcDerivativeValue();
    this->predictValue();

    const double activation = this->getActivationThreshold();
    const double critical = activation * 10.0;
    const double partial_val_sum = this->partial_values_.sum();

    if(this->state_.getCurrent() == CRITICAL && this->prediction_value_ > partial_val_sum && this->pred_active_idx_.size() > 0)
    {
        ROS_WARN_STREAM("Current state is CRITICAL but prediction is > value before -> Stay in CRIT.");
        // Stay in critical but use prediction
        this->partial_values_ = this->predict_partial_values_;
        this->derivative_values_ = this->predict_derivative_values_;
        this->active_idx_ = this->pred_active_idx_;
    }
    else if(this->partial_values_.sum() > critical && this->partial_values_.sum() > 0.0)
    {
        this->state_.setState(CRITICAL); // always highest task -> avoid HW destruction.
    }
    else
    {
        this->state_.setState(DANGER); // always highest task -> avoid HW destruction.
    }
}

/// Calculate values of the JLA cost function.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::calcValue()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;
    const KDL::JntArray joint_pos = this->joint_states_.current_q_;
    this->values_.resize(joint_pos.rows(), 1);
    for(uint8_t i = 0; i < joint_pos.rows() ; ++i)
    {
        double nom = pow(limits_max[i] - limits_min[i], 2.0);
        double denom = (limits_max[i] - joint_pos(i)) * (joint_pos(i) - limits_min[i]);
        this->values_(i) = nom / (4.0 * denom); // store value of cost function for each joint.
    }

    this->last_value_ = this->value_;
    this->value_ = this->values_.sum();
    return this->value_;
}

/// Calculate derivative of values.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::calcDerivativeValue()
{
//    double current_time = ros::Time::now().toSec();
//    double cycle = current_time - this->last_time_;
//    this->last_time_ = current_time;
//    const uint32_t values_rows = this->values_.rows();
//    if(cycle <= 0.0)
//    {
//        cycle = DEFAULT_CYCLE;
//    }
//
//    if(values_rows == this->last_values_.rows())
//    {
//        this->derivative_values_.resize(this->active_idx_.size(), 1);
//        for(uint8_t i = 0; i < this->active_idx_.size(); ++i)
//        {
//            uint8_t active_idx = this->active_idx_[i];
//            double delta = this->values_(active_idx) - this->last_values_(active_idx);
//            if(delta < 1.0e-5)
//            {
//                this->derivative_values_(i) = 0.0;
//            }
//            else
//            {
//                this->derivative_values_(i) = delta / cycle;
//            }
//
//        }
//    }
//    else
//    {
//        uint8_t new_size = this->active_idx_.size() > 0 ? this->active_idx_.size() : values_rows;
//        this->derivative_values_ = Eigen::VectorXd::Zero(new_size, 1);
//    }



    const uint32_t values_rows = this->values_.rows();
    this->derivative_values_.resize(this->active_idx_.size(), 1);
    for(uint8_t i = 0; i < this->active_idx_.size(); ++i)
    {
        uint8_t active_idx = this->active_idx_[i];
        this->derivative_values_(i) = this->exp_decay_ * this->values_(active_idx);
    }

    this->derivative_value_ = this->derivative_values_.sum();
    return this->derivative_value_;
}

/// Calculates values of the gradient of the cost function
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidance<T_PARAMS, PRIO>::calcPartialValues()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    const KDL::JntArray joint_pos = this->joint_states_.current_q_;
    const KDL::JntArray joint_vel = this->joint_states_.current_q_dot_;
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;
    uint8_t vec_rows = static_cast<uint8_t>(joint_pos.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    uint8_t used = 0;
    this->active_idx_.clear();
    for(uint8_t i = 0; i < vec_rows; ++i)
    {
        //See Chan paper ISSN 1042-296X [Page 288]
        if( (joint_vel(i) > 0.0 && ((limits_max[i] - joint_pos(i)) < (joint_pos(i) - limits_min[i])))
                || (joint_vel(i) < 0.0 && ((limits_max[i] - joint_pos(i)) > (joint_pos(i) - limits_min[i]))) )
        {
            double min_delta = (joint_pos(i) - limits_min[i]);
            double max_delta = (limits_max[i] - joint_pos(i));
            double nominator = (2.0 * joint_pos(i) - limits_min[i] - limits_max[i]) * (limits_max[i] - limits_min[i]) * (limits_max[i] - limits_min[i]);
            double denom = 4.0 * min_delta * min_delta * max_delta * max_delta;
            partial_values(used) = nominator / denom;
            ++used;
            this->active_idx_.push_back(i);
        }
    }

    this->partial_values_ = partial_values;
    return this->partial_values_;
}


/// Returns the threshold of the cost function to become active.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getActivationThreshold() const
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    return params.activation_threshold_jla;
}

/// Returns a value for k_H to weight the partial values for GPM e.g.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    double k_H = params.k_H_jla;
    return k_H;
}

template <typename T_PARAMS, typename PRIO>
ConstraintTypes JointLimitAvoidance<T_PARAMS, PRIO>::getType() const
{
    return JLA;
}

/**
 *
 * @return
 */
template <typename T_PARAMS, typename PRIO>
Eigen::MatrixXd JointLimitAvoidance<T_PARAMS, PRIO>::getTaskJacobian() const
{
    Eigen::MatrixXd task_jacobian = Eigen::MatrixXd::Zero(this->active_idx_.size(), this->partial_values_.rows());
            //this->partial_values_.asDiagonal();


    for(uint8_t i = 0; i < this->active_idx_.size(); ++i)
    {
        uint8_t active_idx = this->active_idx_[i];
        task_jacobian(i, active_idx) = this->partial_values_(i);
    }

    ROS_INFO_STREAM("TASK JAC: " << std::endl << task_jacobian);

    return task_jacobian;
}

/**
 *
 * @return
 */
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidance<T_PARAMS, PRIO>::getTaskDerivatives() const
{
    ROS_INFO_STREAM("TASK DERIVS: " << std::endl << this->derivative_values_);

    return this->derivative_values_;
}

template <typename T_PARAMS, typename PRIO>
Task_t JointLimitAvoidance<T_PARAMS, PRIO>::createTask()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    TwistControllerParams adapted_params;
    adapted_params.damping_method = CONSTANT;
    adapted_params.damping_factor = params.damping_jla;
    adapted_params.eps_truncation = 0.0;
    adapted_params.numerical_filtering = false;

    Eigen::MatrixXd cost_func_jac = this->getTaskJacobian();
    Eigen::VectorXd derivs = this->getTaskDerivatives();

    Task_t task(this->getPriority(),
                this->getTaskId(),
                cost_func_jac,
                derivs,
                this->getType());

    task.tcp_ = adapted_params;
    task.db_ = boost::shared_ptr<DampingBase>(DampingBuilder::createDamping(adapted_params));
    return task;
}

/* END JointLimitAvoidance **************************************************************************************/

/* BEGIN 2nd JointLimitAvoidance ************************************************************************************/
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
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getActivationGain() const
{
    return 1.0;
}

template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidanceMid<T_PARAMS, PRIO>::calculate()
{
    this->calcValue();
    this->calcDerivativeValue();
    this->calcPartialValues();
}

/// Calculate values of the JLA cost function.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::calcValue()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;
    const KDL::JntArray joint_pos = this->joint_states_.current_q_;
    double H_q = 0.0;
    for(uint8_t i = 0; i < joint_pos.rows() ; ++i)
    {
        double jnt_pos_with_step = joint_pos(i);
        double nom = pow(limits_max[i] - limits_min[i], 2.0);
        double denom = (limits_max[i] - jnt_pos_with_step) * (jnt_pos_with_step - limits_min[i]);
        H_q += nom / denom;
    }

    this->value_ = H_q / 4.0;
    return this->value_;
}

/// Calculate derivative of values.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::calcDerivativeValue()
{
    double current_time = ros::Time::now().toSec();
    double cycle = current_time - this->last_time_;
    if(cycle > 0.0)
    {
        this->derivative_value_ = (this->value_ - this->last_value_) / cycle;
    }
    else
    {
        this->derivative_value_ = (this->value_ - this->last_value_) / DEFAULT_CYCLE;
    }

    this->last_time_ = current_time;
    return this->derivative_value_;
}

/// Calculates values of the gradient of the cost function
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidanceMid<T_PARAMS, PRIO>::calcPartialValues()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    const KDL::JntArray joint_pos = this->joint_states_.current_q_;
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;

    uint8_t vec_rows = static_cast<uint8_t>(joint_pos.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());

    for(uint8_t i = 0; i < vec_rows; ++i) // max limiting the arm joints but not the extensions.
    {
        double min_delta = joint_pos(i) - limits_min[i];
        double max_delta = limits_max[i] - joint_pos(i);
        if( min_delta * max_delta < 0.0)
        {
            ROS_WARN_STREAM("Limit of joint " << int(i) << " reached: " << std::endl
                            << "pos=" << joint_pos(i) << ";lim_min=" << limits_min[i] << ";lim_max=" << limits_max[i]);
        }

        //Liegeois method can also be found in Chan paper ISSN 1042-296X [Page 288]
        double limits_mid = 1.0 / 2.0 * (limits_max[i] + limits_min[i]);
        double nominator = joint_pos(i) - limits_mid;
        double denom = pow(limits_max[i] - limits_min[i], 2.0);
        partial_values(i) = nominator / denom;
    }

    this->partial_values_ = partial_values;
    return this->partial_values_;
}


/// Returns the threshold of the cost function to become active.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getActivationThreshold() const
{
    return 0.0;
}

/// Returns a value for k_H to weight the partial values for GPM e.g.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    double k_H = params.k_H_jla;
    return k_H;
}

template <typename T_PARAMS, typename PRIO>
ConstraintTypes JointLimitAvoidanceMid<T_PARAMS, PRIO>::getType() const
{
    return JLA_MID;
}
/* END 2nd JointLimitAvoidance **************************************************************************************/

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
double JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getActivationGain() const
{
    const double activation_threshold = this->getActivationThreshold();  // %
    const double activation_buffer_region = activation_threshold * (1.0 + ACTIVATION_BUFFER); // %
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
    else if(rel_delta < activation_buffer_region) // activation_buffer_region == d_i
    {
        activation_gain = 0.5 * (1.0 - cos(M_PI * (rel_delta - activation_threshold) / (activation_buffer_region - activation_threshold)));
    }
    else
    {
        activation_gain = 0.0;
    }

    if(activation_gain < 0.0)
    {
        activation_gain = 0.0;
    }

    return activation_gain;
}

template <typename T_PARAMS, typename PRIO>
void JointLimitAvoidanceIneq<T_PARAMS, PRIO>::calculate()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    const int32_t joint_idx = this->constraint_params_.joint_idx_;
    const double limit_min = std::abs(params.limits_min[joint_idx]);
    const double limit_max = std::abs(params.limits_max[joint_idx]);
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

    double activation = this->getActivationThreshold();
    double critical = 0.4 * activation;

    if(this->state_.getCurrent() == CRITICAL && this->prediction_value_ < rel_val)
    {
        ROS_WARN_STREAM("JLA_Current state is CRITICAL but prediction is smaller than current rel_val -> Stay in CRIT.");
    }
    else if(rel_val < critical || this->prediction_value_ < critical)
    {
        if(this->prediction_value_ < critical)
        {
            ROS_WARN_STREAM("pred_val < critical!!!");
        }

        this->state_.setState(CRITICAL); // -> avoid HW destruction.
    }
    else
    {
        this->state_.setState(DANGER); // GPM always active!
    }
//    else if(rel_val < activation)
//    {
//        this->state_.setState(DANGER); // GPM
//    }
//    else
//    {
//        this->state_.setState(NORMAL);
//    }
}

/// Calculate values of the JLA cost function.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceIneq<T_PARAMS, PRIO>::calcValue()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    int32_t joint_idx = this->constraint_params_.joint_idx_;
    double limit_min = params.limits_min[joint_idx];
    double limit_max = params.limits_max[joint_idx];

    double joint_pos = this->joint_states_.current_q_(joint_idx);

    this->last_value_ = this->value_;
    this->value_ = (limit_max - joint_pos) * (joint_pos - limit_min);
    return this->value_;
}

/**
 * Simple first order differential equation for exponential increase (move away from limit!)
 * @return 1st order diff equation value.
 */
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceIneq<T_PARAMS, PRIO>::calcDerivativeValue()
{
    this->derivative_value_ = 0.1 * this->value_;
    return this->derivative_value_;
}

/// Calculates values of the gradient of the cost function
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidanceIneq<T_PARAMS, PRIO>::calcPartialValues()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    int32_t joint_idx = this->constraint_params_.joint_idx_;
    double limit_min = params.limits_min[joint_idx];
    double limit_max = params.limits_max[joint_idx];
    double joint_pos = this->joint_states_.current_q_(joint_idx);
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(this->jacobian_data_.cols());
    partial_values(this->constraint_params_.joint_idx_) = -2.0 * joint_pos + limit_max + limit_min;
    this->partial_values_ = partial_values;
    return this->partial_values_;
}


/// Returns the threshold of the cost function to become active.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getActivationThreshold() const
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    return params.activation_threshold_jla;
}

/// Returns a value for k_H to weight the partial values for GPM e.g.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    double factor;
    const TwistControllerParams& params = this->constraint_params_.getParams();
    if(this->abs_delta_max_ > this->abs_delta_min_ && this->rel_min_ > 0.0)
    {
        factor = (this->getActivationThreshold() * 1.1 / this->rel_min_) - 1.0;
    }
    else
    {
        if(this->rel_max_ > 0.0)
        {
            factor = (this->getActivationThreshold() * 1.1 / this->rel_max_) - 1.0;
        }
        else
        {
            factor = 1.0; // Suggestion
        }
    }

    double k_H = factor * params.k_H_jla;
    return k_H;
}

template <typename T_PARAMS, typename PRIO>
ConstraintTypes JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getType() const
{
    return JLA_INEQ;
}

/**
 *
 * @return
 */
template <typename T_PARAMS, typename PRIO>
Eigen::MatrixXd JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getTaskJacobian() const
{
    return this->partial_values_.transpose();
}

/**
 *
 * @return
 */
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidanceIneq<T_PARAMS, PRIO>::getTaskDerivatives() const
{
    return Eigen::VectorXd::Identity(1, 1) * this->derivative_value_;
}

template <typename T_PARAMS, typename PRIO>
Task_t JointLimitAvoidanceIneq<T_PARAMS, PRIO>::createTask()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    TwistControllerParams adapted_params;
    adapted_params.damping_method = CONSTANT;
    adapted_params.damping_factor = params.damping_jla;
    adapted_params.eps_truncation = 0.0;
    adapted_params.numerical_filtering = false;

    Eigen::MatrixXd cost_func_jac = this->getTaskJacobian();
    Eigen::VectorXd derivs = this->getTaskDerivatives();

    Task_t task(this->getPriority(),
                this->getTaskId(),
                cost_func_jac,
                derivs,
                this->getType());

    task.tcp_ = adapted_params;
    task.db_ = boost::shared_ptr<DampingBase>(DampingBuilder::createDamping(adapted_params));
    return task;
}

/* END JointLimitAvoidance **************************************************************************************/

#endif /* CONSTRAINT_IMPL_H_ */
