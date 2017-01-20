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
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This module contains the implementation of all interface types.
 *
 ****************************************************************/

#include <pluginlib/class_list_macros.h>
#include "cob_twist_controller/controller_interfaces/controller_interface.h"
#include "cob_twist_controller/controller_interfaces/controller_interface_base.h"

namespace cob_twist_controller
{

/* BEGIN ControllerInterfaceVelocity ********************************************************************************************/
void ControllerInterfaceVelocity::initialize(ros::NodeHandle& nh,
                                             const TwistControllerParams& params)
{
    nh_ = nh;
    params_ = params;
    pub_ = nh.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);
}
/**
 * Method processing the result by publishing to the 'joint_group_velocity_controller/command' topic.
 */
inline void ControllerInterfaceVelocity::processResult(const KDL::JntArray& q_dot_ik,
                                                       const KDL::JntArray& current_q)
{
    std_msgs::Float64MultiArray vel_msg;

    for (unsigned int i = 0; i < params_.dof; i++)
    {
        vel_msg.data.push_back(q_dot_ik(i));
    }

    pub_.publish(vel_msg);
}
/* END ControllerInterfaceVelocity **********************************************************************************************/


/* BEGIN ControllerInterfacePosition ****************************************************************************************/
void ControllerInterfacePosition::initialize(ros::NodeHandle& nh,
                                             const TwistControllerParams& params)
{
    nh_ = nh;
    params_ = params;
    last_update_time_ = ros::Time(0.0);
    integrator_.reset(new SimpsonIntegrator(params.dof, params.integrator_smoothing));
    pub_ = nh.advertise<std_msgs::Float64MultiArray>("joint_group_position_controller/command", 1);
}
/**
 * Method processing the result using integration method (Simpson) and publishing to the 'joint_group_position_controller/command' topic.
 */
inline void ControllerInterfacePosition::processResult(const KDL::JntArray& q_dot_ik,
                                                       const KDL::JntArray& current_q)
{
    if (updateIntegration(q_dot_ik, current_q))
    {
        /// publish to interface
        std_msgs::Float64MultiArray pos_msg;
        pos_msg.data = pos_;
        pub_.publish(pos_msg);
    }
}
/* END ControllerInterfacePosition ******************************************************************************************/


/* BEGIN ControllerInterfaceTrajectory ****************************************************************************************/
void ControllerInterfaceTrajectory::initialize(ros::NodeHandle& nh,
                                               const TwistControllerParams& params)
{
    nh_ = nh;
    params_ = params;
    last_update_time_ = ros::Time(0.0);
    integrator_.reset(new SimpsonIntegrator(params.dof, params.integrator_smoothing));
    pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory_controller/command", 1);
}
/**
 * Method processing the result using integration method (Simpson) and publishing to the 'joint_trajectory_controller/command' topic.
 */
inline void ControllerInterfaceTrajectory::processResult(const KDL::JntArray& q_dot_ik,
                                                         const KDL::JntArray& current_q)
{
    if (updateIntegration(q_dot_ik, current_q))
    {
        trajectory_msgs::JointTrajectoryPoint traj_point;
        traj_point.positions = pos_;
        // traj_point.velocities = vel_;
        // traj_point.accelerations.assign(params_.dof, 0.0);
        // traj_point.effort.assign(params_.dof, 0.0);
        traj_point.time_from_start = period_;

        trajectory_msgs::JointTrajectory traj_msg;
        // traj_msg.header.stamp = ros::Time::now();
        traj_msg.joint_names = params_.joints;
        traj_msg.points.push_back(traj_point);

        /// publish to interface
        pub_.publish(traj_msg);
    }
}
/* END ControllerInterfaceTrajectory ******************************************************************************************/


/* BEGIN ControllerInterfaceJointStates ****************************************************************************************/
void ControllerInterfaceJointStates::initialize(ros::NodeHandle& nh,
                                                const TwistControllerParams& params)
{
    nh_ = nh;
    params_ = params;
    last_update_time_ = ros::Time(0.0);
    integrator_.reset(new SimpsonIntegrator(params.dof, params.integrator_smoothing));
    pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    js_msg_.name = params_.joints;
    js_msg_.position.clear();
    js_msg_.velocity.clear();
    js_msg_.effort.clear();

    for (unsigned int i=0; i < params_.joints.size(); i++)
    {
        // start at center configuration
        js_msg_.position.push_back(params_.limiter_params.limits_min[i] + (params_.limiter_params.limits_max[i] - params_.limiter_params.limits_min[i])/2.0);
        js_msg_.velocity.push_back(0.0);
        js_msg_.effort.push_back(0.0);
    }

    js_timer_ = nh.createTimer(ros::Duration(1/50.0), &ControllerInterfaceJointStates::publishJointState, this);
    js_timer_.start();
}
/**
 * Method processing the result using integration method (Simpson) updating the internal JointState.
 */
inline void ControllerInterfaceJointStates::processResult(const KDL::JntArray& q_dot_ik,
                                                          const KDL::JntArray& current_q)
{
    if (updateIntegration(q_dot_ik, current_q))
    {
        /// update JointState
        boost::mutex::scoped_lock lock(mutex_);
        js_msg_.position = pos_;
        js_msg_.velocity = vel_;

        /// publishing takes place in separate thread
    }
}

/**
 * Timer callback publishing the internal JointState to the 'joint_state' topic.
 */
void ControllerInterfaceJointStates::publishJointState(const ros::TimerEvent& event)
{
    boost::mutex::scoped_lock lock(mutex_);
    js_msg_.header.stamp = ros::Time::now();
    pub_.publish(js_msg_);
}
/* END ControllerInterfaceJointStates ******************************************************************************************/

}

PLUGINLIB_EXPORT_CLASS( cob_twist_controller::ControllerInterfaceVelocity, cob_twist_controller::ControllerInterfaceBase)
PLUGINLIB_EXPORT_CLASS( cob_twist_controller::ControllerInterfacePosition, cob_twist_controller::ControllerInterfaceBase)
PLUGINLIB_EXPORT_CLASS( cob_twist_controller::ControllerInterfaceTrajectory, cob_twist_controller::ControllerInterfaceBase)
PLUGINLIB_EXPORT_CLASS( cob_twist_controller::ControllerInterfaceJointStates, cob_twist_controller::ControllerInterfaceBase)
