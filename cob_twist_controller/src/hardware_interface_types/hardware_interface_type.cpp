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
    ros::Time now = ros::Time::now();
    ros::Duration period = now - last_update_time_;
    last_update_time_ = now;

    if(!vel_before_last_.empty())
    {
        std_msgs::Float64MultiArray pos_msg;
        for(unsigned int i = 0; i < params_.dof; ++i)
        {
            // Simpson
            double integration_value = static_cast<double>(period.toSec() / 6.0 * (vel_before_last_[i] + 4.0 * (vel_before_last_[i] + vel_last_[i]) + vel_before_last_[i] + vel_last_[i] + q_dot_ik(i)) + current_q(i));
            ma_[i].addElement(integration_value);
            double avg = 0.0;
            ma_[i].calcWeightedMovingAverage(avg);
            pos_msg.data.push_back(avg);
        }
        //publish to interface
        pub_.publish(pos_msg);
    }

    // Continuously shift the vectors for simpson integration
    vel_before_last_.clear();
    for(unsigned int i=0; i < vel_last_.size(); ++i)
    {
        vel_before_last_.push_back(vel_last_[i]);
    }

    vel_last_.clear();
    for(unsigned int i=0; i < q_dot_ik.rows(); ++i)
    {
        vel_last_.push_back(q_dot_ik(i));
    }
}
/* END HardwareInterfacePosition ******************************************************************************************/


/* BEGIN HardwareInterfaceJointStates ****************************************************************************************/
/**
 * Method processing the result using integration method (Simpson) updating the internal JointState.
 */
inline void HardwareInterfaceJointStates::processResult(const KDL::JntArray& q_dot_ik,
                                                        const KDL::JntArray& current_q)
{
    ros::Time now = ros::Time::now();
    ros::Duration period = now - last_update_time_;
    last_update_time_ = now;

    if(!vel_before_last_.empty())
    {
        std_msgs::Float64MultiArray vel_msg, pos_msg;
        for(unsigned int i = 0; i < params_.dof; ++i)
        {
            // Simpson
            double integration_value = static_cast<double>(period.toSec() / 6.0 * (vel_before_last_[i] + 4.0 * (vel_before_last_[i] + vel_last_[i]) + vel_before_last_[i] + vel_last_[i] + q_dot_ik(i)) + current_q(i));
            ma_[i].addElement(integration_value);
            double avg = 0.0;
            ma_[i].calcWeightedMovingAverage(avg);
            pos_msg.data.push_back(avg);

            vel_msg.data.push_back(q_dot_ik(i));
        }
        ///update JointState
        boost::mutex::scoped_lock lock(mutex_);
        js_msg_.position = pos_msg.data;
        js_msg_.velocity = vel_msg.data;

        ///publishing takes place in separate thread
    }

    // Continuously shift the vectors for simpson integration
    vel_before_last_.clear();
    for(unsigned int i=0; i < vel_last_.size(); ++i)
    {
        vel_before_last_.push_back(vel_last_[i]);
    }

    vel_last_.clear();
    for(unsigned int i=0; i < q_dot_ik.rows(); ++i)
    {
        vel_last_.push_back(q_dot_ik(i));
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
