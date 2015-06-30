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

#include "cob_twist_controller/interface_types/interface_type.h"



InterfaceBase* InterfaceBuilder::create_interface(ros::NodeHandle& nh,
                                                  const TwistControllerParams &params)
{
    InterfaceBase *ib = NULL;
    switch(params.interface_type)
    {
        case VELOCITY:
            ib = new Interface_Velocity(nh, params);
            break;
        case POSITION:
            ib = new Interface_Position(nh, params);
            break;
        default:
            ib = new Interface_Velocity(nh, params);
            break;
    }

    return ib;
}

/* BEGIN Interface_Velocity ********************************************************************************************/
inline void Interface_Velocity::process_result(const KDL::JntArray &q_dot_ik,
                                               std::vector<double> &initial_position)
{
    std_msgs::Float64MultiArray vel_msg;

    for(unsigned int i = 0; i < params_.dof; i++)
    {
        vel_msg.data.push_back(q_dot_ik(i));
    }

    pub_.publish(vel_msg);
}

/* END Interface_Velocity **********************************************************************************************/



/* BEGIN Interface_Position ****************************************************************************************/
inline void Interface_Position::process_result(const KDL::JntArray &q_dot_ik,
                                               std::vector<double> &initial_position)
{
    std_msgs::Float64MultiArray vel_msg, pos_msg;

    time_now_ = ros::Time::now();
    integration_period_ = time_now_ - last_update_time_;

    for(unsigned int i = 0; i < params_.dof; ++i)
    {
        vel_msg.data.push_back(q_dot_ik(i));

           // Simpson
           if(iteration_counter_>1)
           {
               double integration_value = static_cast<double>(integration_period_.toSec() / 6.0 * (vel_first_integration_point_[i] + 4.0 * (vel_first_integration_point_[i] + vel_support_integration_point_[i]) + vel_first_integration_point_[i] + vel_support_integration_point_[i] + vel_msg.data[i]) + initial_position[i]);
               ma_[i].add_element(integration_value);
               pos_msg.data.push_back(ma_[i].calc_weighted_moving_average());
           }
    }
    last_update_time_ = time_now_;

    // Continuously shift the vectors for simpson integration
    if(iteration_counter_ > 1)
    {
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
    }

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

        pub_.publish(pos_msg);
    }

    if(iteration_counter_ < 3)
    {
        ++iteration_counter_;
    }
}

/* END Interface_Position ******************************************************************************************/
