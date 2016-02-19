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

#include "cob_twist_controller/controller_interfaces/controller_interface.h"

/* BEGIN ControllerInterfaceBuilder *****************************************************************************************/
/**
 * Static builder method to create controller interface based on given parameterization.
 */
ControllerInterfaceBase* ControllerInterfaceBuilder::createControllerInterface(ros::NodeHandle& nh,
                                                                               const TwistControllerParams& params,
                                                                               const JointStates& joint_states)
{
    ControllerInterfaceBase* ib = NULL;
    switch (params.controller_interface)
    {
        case VELOCITY_INTERFACE:
            ib = new ControllerInterfaceVelocity(nh, params);
            break;
        case POSITION_INTERFACE:
            ib = new ControllerInterfacePosition(nh, params);
            break;
        case TRAJECTORY_INTERFACE:
            ib = new ControllerInterfaceTrajectory(nh, params);
            break;
        case JOINT_STATE_INTERFACE:
            ib = new ControllerInterfaceJointStates(nh, params, joint_states);
            break;
        default:
            ROS_ERROR("ControllerInterface %d not defined! Using default: 'VELOCITY_INTERFACE'!", params.controller_interface);
            ib = new ControllerInterfaceVelocity(nh, params);
            break;
    }

    return ib;
}
/* END ControllerInterfaceBuilder *******************************************************************************************/

/* BEGIN ControllerInterfaceVelocity ********************************************************************************************/
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
