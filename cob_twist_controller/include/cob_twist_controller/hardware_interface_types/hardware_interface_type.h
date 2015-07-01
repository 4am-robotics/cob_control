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
#ifndef COB_CONTROL_COB_TWIST_CONTROLLER_INCLUDE_INTERFACE_TYPES_INTERFACE_TYPE_H_
#define COB_CONTROL_COB_TWIST_CONTROLLER_INCLUDE_INTERFACE_TYPES_INTERFACE_TYPE_H_

#include "cob_twist_controller/hardware_interface_types/hardware_interface_type_base.h"

/* BEGIN HardwareInterfaceBuilder *****************************************************************************************/
/// Class providing a static method to create hardware interface objects.
class HardwareInterfaceBuilder
{
    public:
        HardwareInterfaceBuilder() {}
        ~HardwareInterfaceBuilder() {}
        
        static HardwareInterfaceBase* create_interface(ros::NodeHandle& nh,
                                               const TwistControllerParams &params);
};
/* END HardwareInterfaceBuilder *******************************************************************************************/




/* BEGIN HardwareInterfaceVelocity ****************************************************************************************/
class HardwareInterfaceVelocity : public HardwareInterfaceBase
{
    public:
        HardwareInterfaceVelocity(ros::NodeHandle& nh, const TwistControllerParams &params)
        : HardwareInterfaceBase(nh, params)
        {
            pub_ = nh.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);
        }

        ~HardwareInterfaceVelocity() {}

        virtual void process_result(const KDL::JntArray &q_dot_ik,
                                    std::vector<double> &initial_position);
};
/* END HardwareInterfaceVelocity **********************************************************************************************/


/* BEGIN HardwareInterfacePosition ****************************************************************************************/
class HardwareInterfacePosition : public HardwareInterfaceBase
{
    public:
        HardwareInterfacePosition(ros::NodeHandle& nh, const TwistControllerParams &params)
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

        virtual void process_result(const KDL::JntArray &q_dot_ik,
                                    std::vector<double> &initial_position);

    private:
        std::vector<MovingAverage> ma_;
        std::vector<double> vel_support_integration_point_, vel_first_integration_point_;
        unsigned int iteration_counter_;
        ros::Duration integration_period_;
        ros::Time time_now_;
        ros::Time last_update_time_;

};
/* END HardwareInterfacePosition **********************************************************************************************/



#endif /* COB_CONTROL_COB_TWIST_CONTROLLER_INCLUDE_INTERFACE_TYPES_INTERFACE_TYPE_H_ */
