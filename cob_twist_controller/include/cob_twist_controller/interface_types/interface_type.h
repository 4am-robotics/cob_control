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
 *   This header contains the class and method definitions of
 *   all interface types (position/velocity).
 *
 ****************************************************************/
#ifndef COB_CONTROL_COB_TWIST_CONTROLLER_INCLUDE_INTERFACE_TYPES_INTERFACE_TYPE_H_
#define COB_CONTROL_COB_TWIST_CONTROLLER_INCLUDE_INTERFACE_TYPES_INTERFACE_TYPE_H_

#include "cob_twist_controller/interface_types/interface_type_base.h"

/* BEGIN InterfaceBuilder *****************************************************************************************/
/// Class providing a static method to create damping method objects.
class InterfaceBuilder
{
    public:
        InterfaceBuilder() {}
        ~InterfaceBuilder() {}
        
        static InterfaceBase* create_interface(ros::NodeHandle& nh,
                                               const TwistControllerParams &params);
};
/* END InterfaceBuilder *******************************************************************************************/




/* BEGIN InterfaceVelocity ****************************************************************************************/
class InterfaceVelocity : public InterfaceBase
{
    public:
        InterfaceVelocity(ros::NodeHandle& nh, const TwistControllerParams &params)
        : InterfaceBase(nh, params)
        {
            pub_ = nh.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);
        }

        ~InterfaceVelocity() {}

        virtual void process_result(const KDL::JntArray &q_dot_ik,
                                    std::vector<double> &initial_position);
};
/* END InterfaceVelocity **********************************************************************************************/


/* BEGIN InterfacePosition ****************************************************************************************/
class InterfacePosition : public InterfaceBase
{
    public:
        InterfacePosition(ros::NodeHandle& nh, const TwistControllerParams &params)
        : InterfaceBase(nh, params),
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

        ~InterfacePosition() {}

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
/* END InterfacePosition **********************************************************************************************/



#endif /* COB_CONTROL_COB_TWIST_CONTROLLER_INCLUDE_INTERFACE_TYPES_INTERFACE_TYPE_H_ */
