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

#include "cob_twist_controller/hardware_interface_types/hardware_interface_type.h"

/* BEGIN HardwareInterfaceBuilder *****************************************************************************************/
/**
 * Static builder method to create hardware interface based on given parameterization.
 */
HardwareInterfaceBase* HardwareInterfaceBuilder::createHardwareInterface(ros::NodeHandle& nh,
                                                                         const TwistControllerParams& params)
{
    HardwareInterfaceBase* ib = NULL;
    switch(params.hardware_interface_type)
    {
        case VELOCITY_INTERFACE:
            ib = new HardwareInterfaceVelocity(nh, params);
            break;
        case POSITION_INTERFACE:
            ib = new HardwareInterfacePosition(nh, params);
            break;
        case TRAJECTORY_INTERFACE:
            ib = new HardwareInterfaceTrajectory(nh, params);
            break;
        case JOINT_STATE_INTERFACE:
            ib = new HardwareInterfaceJointStates(nh, params);
            break;
        default:
            ROS_ERROR("HardwareInterface %d not defined! Using default: 'VELOCITY_INTERFACE'!", params.hardware_interface_type);
            ib = new HardwareInterfaceVelocity(nh, params);
            break;
    }

    return ib;
}
/* END HardwareInterfaceBuilder *******************************************************************************************/

/* BEGIN HardwareInterfaceVelocity ********************************************************************************************/
/**
 * Method processing the result by publishing to the 'joint_group_velocity_controller/command' topic.
 */
inline void HardwareInterfaceVelocity::processResult(const KDL::JntArray& q_dot_ik,
                                                     const KDL::JntArray& current_q)
{
    std_msgs::Float64MultiArray vel_msg;

    for(unsigned int i = 0; i < params_.dof; i++)
    {
        vel_msg.data.push_back(q_dot_ik(i));
    }

    pub_.publish(vel_msg);
}
/* END HardwareInterfaceVelocity **********************************************************************************************/


/* BEGIN HardwareInterfacePosition ****************************************************************************************/
/**
 * Method processing the result using integration method (Simpson) and publishing to the 'joint_group_position_controller/command' topic.
 */
inline void HardwareInterfacePosition::processResult(const KDL::JntArray& q_dot_ik,
                                                     const KDL::JntArray& current_q)
{
    if(updateIntegration(q_dot_ik, current_q))
    {
        ///publish to interface
        std_msgs::Float64MultiArray pos_msg;
        pos_msg.data = pos;
        pub_.publish(pos_msg);
    }
}
/* END HardwareInterfacePosition ******************************************************************************************/


/* BEGIN HardwareInterfaceTrajectory ****************************************************************************************/
/**
 * Method processing the result using integration method (Simpson) and publishing to the 'joint_trajectory_controller/command' topic.
 */
inline void HardwareInterfaceTrajectory::processResult(const KDL::JntArray& q_dot_ik,
                                                       const KDL::JntArray& current_q)
{
    if(updateIntegration(q_dot_ik, current_q))
    {
        ///publish to interface
        trajectory_msgs::JointTrajectoryPoint traj_point;
        traj_point.positions = pos;
        traj_point.velocities = vel;
        traj_point.accelerations.assign(params_.dof, 0.0);
        traj_point.effort.assign(params_.dof, 0.0);
        traj_point.time_from_start = now_ - last_update_time_;  //Maybe we need a longer time_from_start?
        
        trajectory_msgs::JointTrajectory traj_msg;
        traj_msg.header.stamp = now_;
        traj_msg.joint_names = params_.joints;
        traj_msg.points.push_back(traj_point);
        
        //publish to interface
        pub_.publish(traj_msg);
    }
}
/* END HardwareInterfaceTrajectory ******************************************************************************************/


/* BEGIN HardwareInterfaceJointStates ****************************************************************************************/
/**
 * Method processing the result using integration method (Simpson) updating the internal JointState.
 */
inline void HardwareInterfaceJointStates::processResult(const KDL::JntArray& q_dot_ik,
                                                        const KDL::JntArray& current_q)
{
    if(updateIntegration(q_dot_ik, current_q))
    {
        ///update JointState
        boost::mutex::scoped_lock lock(mutex_);
        js_msg_.position = pos;
        js_msg_.velocity = vel;

        ///publishing takes place in separate thread
    }
}

void HardwareInterfaceJointStates::publishJointState(const ros::TimerEvent& event)
/**
 * Timer callback publishing the internal JointState to the 'joint_state' topic.
 */
{
    boost::mutex::scoped_lock lock(mutex_);
    js_msg_.header.stamp = ros::Time::now();
    pub_.publish(js_msg_);
}

/* END HardwareInterfaceJointStates ******************************************************************************************/
