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
 *   controller interfaces (Velocity/Position/Trajectory/JointStates).
 *
 ****************************************************************/
#ifndef CONTROLLER_INTERFACE_H_
#define CONTROLLER_INTERFACE_H_

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <boost/thread/mutex.hpp>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/utils/moving_average.h"

#include "cob_twist_controller/controller_interfaces/controller_interface_base.h"

/* BEGIN ControllerInterfaceBuilder *****************************************************************************************/
/// Class providing a static method to create controller interface objects.
class ControllerInterfaceBuilder
{
    public:
        ControllerInterfaceBuilder() {}
        ~ControllerInterfaceBuilder() {}

        static ControllerInterfaceBase* createControllerInterface(ros::NodeHandle& nh,
                                               const TwistControllerParams& params);
};
/* END ControllerInterfaceBuilder *******************************************************************************************/


/* BEGIN ControllerInterfaceVelocity ****************************************************************************************/
/// Class providing a ControllerInterface publishing velocities.
class ControllerInterfaceVelocity : public ControllerInterfaceBase
{
    public:
        ControllerInterfaceVelocity(ros::NodeHandle& nh,
                                    const TwistControllerParams& params)
        : ControllerInterfaceBase(nh, params)
        {
            pub_ = nh.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);
        }

        ~ControllerInterfaceVelocity() {}

        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);
};
/* END ControllerInterfaceVelocity **********************************************************************************************/


/* BEGIN ControllerInterfacePosition ****************************************************************************************/
/// Class providing a ControllerInterface publishing JointGroupPositionCommands.
class ControllerInterfacePosition : public ControllerInterfacePositionBase
{
    public:
        ControllerInterfacePosition(ros::NodeHandle& nh,
                                    const TwistControllerParams& params)
        : ControllerInterfacePositionBase(nh, params, 3)
        {
            pub_ = nh.advertise<std_msgs::Float64MultiArray>("joint_group_interpol_position_controller/command", 1);
        }

        ~ControllerInterfacePosition() {}

        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);

};
/* END ControllerInterfacePosition **********************************************************************************************/


/* BEGIN ControllerInterfaceTrajectory ****************************************************************************************/
/// Class providing a ControllerInterface publishing a JointTrajectory.
class ControllerInterfaceTrajectory : public ControllerInterfacePositionBase
{
    public:
        ControllerInterfaceTrajectory(ros::NodeHandle& nh,
                                      const TwistControllerParams& params)
        : ControllerInterfacePositionBase(nh, params, 3)
        {
            pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory_controller/command", 1);
        }

        ~ControllerInterfaceTrajectory() {}

        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);
};
/* END ControllerInterfaceTrajectory **********************************************************************************************/

/* BEGIN ControllerInterfaceJointStates ****************************************************************************************/
/// Class providing a ControllerInterface publishing JointStates.
class ControllerInterfaceJointStates : public ControllerInterfacePositionBase
{
    public:
        ControllerInterfaceJointStates(ros::NodeHandle& nh,
                                       const TwistControllerParams& params)
        : ControllerInterfacePositionBase(nh, params, 3)
        {
            pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

            js_msg_.name = params_.joints;
            js_msg_.position.assign(params_.joints.size(), 0.0);
            js_msg_.velocity.assign(params_.joints.size(), 0.0);
            js_msg_.effort.assign(params_.joints.size(), 0.0);

            js_timer_ = nh.createTimer(ros::Duration(1/60.0), &ControllerInterfaceJointStates::publishJointState, this);
            js_timer_.start();
        }

        ~ControllerInterfaceJointStates() {}

        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);

    private:
        boost::mutex mutex_;
        sensor_msgs::JointState js_msg_;

        ros::Timer js_timer_;
        void publishJointState(const ros::TimerEvent& event);

};
/* END ControllerInterfaceJointStates **********************************************************************************************/


#endif /* CONTROLLER_INTERFACE_H_ */
