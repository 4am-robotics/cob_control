/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


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
        // reflect the joint_state_publisher behavior
        double pos = 0.0;
        if (params_.limiter_params.limits_min[i] > 0.0 || params_.limiter_params.limits_max[i] < 0.0)
        {
            ROS_WARN("Zero is not within JointLimits [%f, %f] of %s! Using mid-configuration", params_.limiter_params.limits_min[i], params_.limiter_params.limits_max[i], params_.joints[i].c_str());
            
            // check whether limits are finite (required when dealing with CONTINUOUS joints)
            if (std::isfinite(params_.limiter_params.limits_min[i]) && std::isfinite(params_.limiter_params.limits_max[i]))
            {
                pos = params_.limiter_params.limits_min[i] + (params_.limiter_params.limits_max[i] - params_.limiter_params.limits_min[i])/2.0;
            }
            else
            {
                ROS_ERROR("JointLimits are infinite (%s is a CONTINUOUS joint)", params_.joints[i].c_str());
            }
        }
        js_msg_.position.push_back(pos);
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
