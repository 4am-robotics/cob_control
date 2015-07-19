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

/* BEGIN HardwareInterfaceVelocity ********************************************************************************************/
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
inline void HardwareInterfacePosition::processResult(const KDL::JntArray& q_dot_ik,
                                                     const KDL::JntArray& current_q)
{
    std_msgs::Float64MultiArray vel_msg, pos_msg;

    time_now_ = ros::Time::now();
    integration_period_ = time_now_ - last_update_time_;

    for(unsigned int i = 0; i < params_.dof; ++i)
    {
        vel_msg.data.push_back(q_dot_ik(i));

        // Simpson
        if(iteration_counter_ > 1)
        {
            double integration_value = static_cast<double>(integration_period_.toSec() / 6.0 * (vel_first_integration_point_[i] + 4.0 * (vel_first_integration_point_[i] + vel_support_integration_point_[i]) + vel_first_integration_point_[i] + vel_support_integration_point_[i] + vel_msg.data[i]) + current_q(i));
            ma_[i].addElement(integration_value);
            pos_msg.data.push_back(ma_[i].calcWeightedMovingAverage());
        }
    }
    last_update_time_ = time_now_;
    
    
    // Initialize the velocity vectors in the first and second iteration
    if(iteration_counter_ == 0)
    {
        vel_first_integration_point_.clear();
        for(int i=0; i < vel_msg.data.size(); ++i)
        {
            vel_first_integration_point_.push_back(vel_msg.data[i]);
        }
    }

    if(iteration_counter_ == 1)
    {
        vel_support_integration_point_.clear();
        for(int i=0; i < vel_msg.data.size(); ++i)
        {
            vel_support_integration_point_.push_back(vel_msg.data[i]);
        }
    }

    if(iteration_counter_ > 1)
    {
        // Continuously shift the vectors for simpson integration
        vel_first_integration_point_.clear();
        for(int i=0; i < vel_support_integration_point_.size(); ++i)
        {
            vel_first_integration_point_.push_back(vel_support_integration_point_[i]);
        }

        vel_support_integration_point_.clear();
        for(int i=0; i < vel_msg.data.size(); ++i)
        {
            vel_support_integration_point_.push_back(vel_msg.data[i]);
        }
        
        //publish to interface
        pub_.publish(pos_msg);
    }

    if(iteration_counter_ < 3)
    {
        ++iteration_counter_;
    }
}
/* END HardwareInterfacePosition ******************************************************************************************/


/* BEGIN HardwareInterfaceJointStates ****************************************************************************************/
inline void HardwareInterfaceJointStates::processResult(const KDL::JntArray& q_dot_ik,
                                                        const KDL::JntArray& current_q)
{
    std_msgs::Float64MultiArray vel_msg, pos_msg;
    sensor_msgs::JointState js_msg;

    time_now_ = ros::Time::now();
    integration_period_ = time_now_ - last_update_time_;

    for(unsigned int i = 0; i < params_.dof; ++i)
    {
        vel_msg.data.push_back(q_dot_ik(i));

        // Simpson
        if(iteration_counter_ > 1)
        {
            double integration_value = static_cast<double>(integration_period_.toSec() / 6.0 * (vel_first_integration_point_[i] + 4.0 * (vel_first_integration_point_[i] + vel_support_integration_point_[i]) + vel_first_integration_point_[i] + vel_support_integration_point_[i] + vel_msg.data[i]) + current_q(i));
            ma_[i].addElement(integration_value);
            pos_msg.data.push_back(ma_[i].calcWeightedMovingAverage());
        }
    }
    last_update_time_ = time_now_;
    
    
    // Initialize the velocity vectors in the first and second iteration
    if(iteration_counter_ == 0)
    {
        vel_first_integration_point_.clear();
        for(int i=0; i < vel_msg.data.size(); ++i)
        {
            vel_first_integration_point_.push_back(vel_msg.data[i]);
        }
    }

    if(iteration_counter_ == 1)
    {
        vel_support_integration_point_.clear();
        for(int i=0; i < vel_msg.data.size(); ++i)
        {
            vel_support_integration_point_.push_back(vel_msg.data[i]);
        }
    }

    if(iteration_counter_ > 1)
    {
        // Continuously shift the vectors for simpson integration
        vel_first_integration_point_.clear();
        for(int i=0; i < vel_support_integration_point_.size(); ++i)
        {
            vel_first_integration_point_.push_back(vel_support_integration_point_[i]);
        }

        vel_support_integration_point_.clear();
        for(int i=0; i < vel_msg.data.size(); ++i)
        {
            vel_support_integration_point_.push_back(vel_msg.data[i]);
        }
        
        //publish to interface
        js_msg.header.stamp = ros::Time::now();
        js_msg.name = params_.joints;
        js_msg.position = pos_msg.data;
        js_msg.velocity = vel_msg.data;
        js_msg.effort.assign(params_.joints.size(), 0.0);
        pub_.publish(js_msg);
    }

    if(iteration_counter_ < 3)
    {
        ++iteration_counter_;
    }
}
/* END HardwareInterfaceJointStates ******************************************************************************************/
