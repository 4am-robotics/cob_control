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
 *   This header contains the interface description of all available
 *   hardware interface types (position/velocity).
 *
 ****************************************************************/
#ifndef HARDWARE_INTERFACE_TYPE_H_
#define HARDWARE_INTERFACE_TYPE_H_

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <boost/thread/mutex.hpp>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/utils/moving_average.h"

#include "cob_twist_controller/hardware_interface_types/hardware_interface_type_base.h"

/* BEGIN HardwareInterfaceBuilder *****************************************************************************************/
/// Class providing a static method to create hardware interface objects.
class HardwareInterfaceBuilder
{
    public:
        HardwareInterfaceBuilder() {}
        ~HardwareInterfaceBuilder() {}
        
        static HardwareInterfaceBase* createHardwareInterface(ros::NodeHandle& nh,
                                               const TwistControllerParams& params);
};
/* END HardwareInterfaceBuilder *******************************************************************************************/


/* BEGIN HardwareInterfaceVelocity ****************************************************************************************/
/// Class providing a HardwareInterface publishing velocities.
class HardwareInterfaceVelocity : public HardwareInterfaceBase
{
    public:
        HardwareInterfaceVelocity(ros::NodeHandle& nh,
                                  const TwistControllerParams& params)
        : HardwareInterfaceBase(nh, params)
        {
            pub_ = nh.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);
        }

        ~HardwareInterfaceVelocity() {}

        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);
};
/* END HardwareInterfaceVelocity **********************************************************************************************/


/* BEGIN HardwareInterfacePosition ****************************************************************************************/
/// Class providing a HardwareInterface publishing positions.
class HardwareInterfacePosition : public HardwareInterfaceBase
{
    public:
        HardwareInterfacePosition(ros::NodeHandle& nh,
                                  const TwistControllerParams& params)
        : HardwareInterfaceBase(nh, params),
          iteration_counter_(0)
        {
            for(int i = 0; i < params.dof; i++)
            {
                ma_.push_back(MovingAverage());
                vel_support_integration_point_.push_back(0.0);
                vel_first_integration_point_.push_back(0.0);
            }
            
            time_now_ = ros::Time::now();
            last_update_time_ = time_now_;
            integration_period_ = time_now_ - last_update_time_;
            pub_ = nh.advertise<std_msgs::Float64MultiArray>("joint_group_position_controller/command", 1);
        }

        ~HardwareInterfacePosition() {}

        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);

    private:
        std::vector<MovingAverage> ma_;
        std::vector<double> vel_support_integration_point_, vel_first_integration_point_;
        unsigned int iteration_counter_;
        ros::Duration integration_period_;
        ros::Time time_now_;
        ros::Time last_update_time_;

};
/* END HardwareInterfacePosition **********************************************************************************************/


/* BEGIN HardwareInterfaceJointStates ****************************************************************************************/
/// Class providing a HardwareInterface publishing joint_states.
class HardwareInterfaceJointStates : public HardwareInterfaceBase
{
    public:
        HardwareInterfaceJointStates(ros::NodeHandle& nh,
                                     const TwistControllerParams& params)
        : HardwareInterfaceBase(nh, params),
          iteration_counter_(0)
        {
            for(int i = 0; i < params.dof; i++)
            {
                ma_.push_back(MovingAverage());
                vel_support_integration_point_.push_back(0.0);
                vel_first_integration_point_.push_back(0.0);
            }
            
            time_now_ = ros::Time::now();
            last_update_time_ = time_now_;
            integration_period_ = time_now_ - last_update_time_;
            pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
            
            js_msg_.name = params_.joints;
            js_msg_.position.assign(params_.joints.size(), 0.0);
            js_msg_.velocity.assign(params_.joints.size(), 0.0);
            js_msg_.effort.assign(params_.joints.size(), 0.0);
            
            js_timer_ = nh.createTimer(ros::Duration(1/60.0), &HardwareInterfaceJointStates::publishJointState, this);
            js_timer_.start();
        }

        ~HardwareInterfaceJointStates() {}

        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);

    private:
        std::vector<MovingAverage> ma_;
        std::vector<double> vel_support_integration_point_, vel_first_integration_point_;
        unsigned int iteration_counter_;
        ros::Duration integration_period_;
        ros::Time time_now_;
        ros::Time last_update_time_;
        
        boost::mutex mutex_;
        sensor_msgs::JointState js_msg_;
        
        ros::Timer js_timer_;
        void publishJointState(const ros::TimerEvent& event);

};
/* END HardwareInterfaceJointStates **********************************************************************************************/


#endif /* HARDWARE_INTERFACE_TYPE_H_ */
