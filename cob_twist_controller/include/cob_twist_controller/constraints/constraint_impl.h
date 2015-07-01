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

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>

#include "cob_twist_controller/constraints/constraint.h"
#include "cob_twist_controller/constraints/constraint_params.h"

#include "cob_twist_controller/damping_methods/damping.h"
#include "cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h"




/* BEGIN ConstraintsBuilder *************************************************************************************/
/**
 * Static builder method to create damping methods dependent on parameterization.
 */
template <typename PRIO>
std::set<tConstraintBase> ConstraintsBuilder<PRIO>::createConstraints(const TwistControllerParams& twist_controller_params,
                                                                       KDL::ChainJntToJacSolver& jnt_to_jac,
                                                                       CallbackDataMediator& data_mediator)
{
    std::set<tConstraintBase> constraints;
    if (GPM_JLA == twist_controller_params.constraint)
    {
        typedef JointLimitAvoidance<ConstraintParamsJLA, PRIO> tJla;
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(twist_controller_params, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<tJla > jla(new tJla(100, params, data_mediator));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if(GPM_JLA_MID == twist_controller_params.constraint)
    {
        typedef JointLimitAvoidanceMid<ConstraintParamsJLA, PRIO> tJlaMid;
        // same params as for normal JLA
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(twist_controller_params, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<tJlaMid > jla(new tJlaMid(100, params, data_mediator));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if(GPM_CA == twist_controller_params.constraint ||
            TASK_STACK_NO_GPM == twist_controller_params.constraint ||
            TASK_STACK_GPM == twist_controller_params.constraint ||
            TASK_2ND_PRIO == twist_controller_params.constraint ||
            DYN_TASKS_READJ == twist_controller_params.constraint)
    {
        typedef CollisionAvoidance<ConstraintParamsCA, PRIO> tCollisionAvoidance;
        uint32_t available_dists = data_mediator.obstacleDistancesCnt();
        uint32_t startPrio = 100;
        for (uint32_t i = 0; i < available_dists; ++i)
        {
            ConstraintParamsCA params = ConstraintParamFactory<ConstraintParamsCA>::createConstraintParams(twist_controller_params, data_mediator);
            // TODO: take care PRIO could be of different type than UINT32
            boost::shared_ptr<tCollisionAvoidance > ca(new tCollisionAvoidance(startPrio--, params, data_mediator, jnt_to_jac));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(ca));
        }
    }
    else
    {
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
    double activation_buffer_region = activation * 1.05;

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
    this->calcValue();
    this->calcDerivativeValue();
    this->calcPartialValues();

    double activation = this->getActivationThreshold();
    double critical = 0.5 * activation;

    if(d.min_distance < critical)
    {
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
    this->value_ = pow(d.min_distance - this->getActivationThreshold(), 2.0);
    return this->value_;
}


template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::calcDerivativeValue()
{
    double current_time = ros::Time::now().toSec();
    double cycle = current_time - this->last_time_;
    if(cycle > 0.0)
    {
        this->derivative_value_ = (this->value_ - this->last_value_) / cycle;
    }
    else
    {
        this->derivative_value_ = (this->value_ - this->last_value_) / 0.02;
    }

    this->last_time_ = current_time;
    return this->derivative_value_;
}

template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd CollisionAvoidance<T_PARAMS, PRIO>::calcPartialValues()
{
    uint8_t vecRows = static_cast<uint8_t>(this->joint_states_.current_q_.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(vecRows);
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
            Eigen::Matrix3d skew_symm;
            skew_symm <<    0.0,                          d.collision_pnt_vector.z(), -d.collision_pnt_vector.y(),
                            -d.collision_pnt_vector.z(),  0.0,                         d.collision_pnt_vector.x(),
                             d.collision_pnt_vector.y(), -d.collision_pnt_vector.x(),  0.0;

            Eigen::Matrix3d ident = Eigen::Matrix3d::Identity();
            Eigen::Matrix<double,6,6> T;
            T.block(0, 0, 3, 3) << ident;
            T.block(0, 3, 3, 3) << skew_symm;
            T.block(3, 0, 3, 3) << Eigen::Matrix3d::Zero();
            T.block(3, 3, 3, 3) << ident;


            uint32_t idx = str_it - params.frame_names.begin();
            uint32_t frame_number = idx + 1; // segment nr not index represents frame number

            KDL::Jacobian new_jac_chain(size_of_frames);

            KDL::JntArray ja = this->joint_states_.current_q_;
            if(0 != this->jnt_to_jac_.JntToJac(ja, new_jac_chain, frame_number))
            {
                ROS_ERROR_STREAM("Failed to calculate JntToJac.");
                return partial_values;
            }

            t_Matrix6Xd crit_pnt_jac = T * new_jac_chain.data;

            Eigen::Matrix3Xd m_transl = Eigen::Matrix3Xd::Zero(3, size_of_frames);
            m_transl << crit_pnt_jac.row(0),
                        crit_pnt_jac.row(1),
                        crit_pnt_jac.row(2);

            Eigen::Vector3d vec;
            vec << d.distance_vec[0], d.distance_vec[1], d.distance_vec[2];
            Eigen::VectorXd term_2nd = (m_transl.transpose()) * (vec / vec.norm()); // use the unit vector only for direction!

            // Gradient of the cost function from: Strasse O., Escande A., Mansard N. et al.
            // "Real-Time (Self)-Collision Avoidance Task on a HRP-2 Humanoid Robot", 2008 IEEE International Conference
            partial_values =  (2.0 * ((d.min_distance - this->getActivationThreshold()) / d.min_distance) * term_2nd);
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
    return 0.1; // in [m]
}


/// Returns a value for k_H to weight the partial values for GPM e.g.
template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    double magnitude = 0.0;
    double activation = this->getActivationThreshold();
    ObstacleDistanceInfo d = this->constraint_params_.current_distance_;

    if (d.min_distance < activation && d.min_distance > 0.0)
    {
        magnitude = pow(activation / d.min_distance, 2.0) - 1.0;
    }

    return -magnitude; // constraint has to be minimized -> therefore minus
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
void JointLimitAvoidance<T_PARAMS, PRIO>::calculate()
{
    this->calcValue();
    this->calcDerivativeValue();
    this->calcPartialValues();
}

/// Calculate values of the JLA cost function.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::calcValue()
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
double JointLimitAvoidance<T_PARAMS, PRIO>::calcDerivativeValue()
{
    double current_time = ros::Time::now().toSec();
    double cycle = current_time - this->last_time_;
    if(cycle > 0.0)
    {
        this->derivative_value_ = (this->value_ - this->last_value_) / cycle;
    }
    else
    {
        this->derivative_value_ = (this->value_ - this->last_value_) / 0.02;
    }

    this->last_time_ = current_time;
    return this->derivative_value_;
}

/// Calculates values of the gradient of the cost function
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidance<T_PARAMS, PRIO>::calcPartialValues()
{
    const TwistControllerParams& params = this->constraint_params_.getParams();
    const KDL::JntArray joint_pos = this->joint_states_.current_q_;
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;
    uint8_t vec_rows = static_cast<uint8_t>(joint_pos.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(vec_rows);
    for(uint8_t i = 0; i < joint_pos.rows() ; ++i)
    {
        partial_values(i) = 0.0; // in the else cases -> output always 0
        //See Chan paper ISSN 1042-296X [Page 288]
        double min_delta = (joint_pos(i) - limits_min[i]);
        double max_delta = (limits_max[i] - joint_pos(i));
        double nominator = (2.0 * joint_pos(i) - limits_min[i] - limits_max[i]) * (limits_max[i] - limits_min[i]) * (limits_max[i] - limits_min[i]);
        double denom = 4.0 * min_delta * min_delta * max_delta * max_delta;
        partial_values(i) = nominator / denom;
    }

    this->partial_values_ = partial_values;
    return this->partial_values_;
}


/// Returns the threshold of the cost function to become active.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getActivationThreshold() const
{
    return 0.0;
}

/// Returns a value for k_H to weight the partial values for GPM e.g.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    // k_H by Armijo-Rule
    double t;
    const TwistControllerParams& params = this->constraint_params_.getParams();
    t = SelfMotionMagnitudeFactory< SmmDeterminatorVelocityBounds<MIN_CRIT> >::calculate(params, particular_solution, homogeneous_solution);
    return t;
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
        this->derivative_value_ = (this->value_ - this->last_value_) / 0.02;
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
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(vec_rows);

    for(uint8_t i = 0; i < vec_rows; ++i)
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
        double denom = limits_max[i] - limits_min[i];
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
    return SelfMotionMagnitudeFactory<SmmDeterminatorVelocityBounds<MAX_CRIT> >::calculate(params, particular_solution, homogeneous_solution);
}
/* END 2nd JointLimitAvoidance **************************************************************************************/

#endif /* CONSTRAINT_IMPL_H_ */
