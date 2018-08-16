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


#include <math.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <cob_tricycle_controller/TricycleCtrlTypes.h>
#include <realtime_tools/realtime_publisher.h>
#include <cob_base_controller_utils/WheelCommands.h>
#include <geometry_msgs/Twist.h>

#include <pluginlib/class_list_macros.h>

namespace cob_tricycle_controller
{

double limitValue(double value, double limit){
    if(limit != 0){
        if (value > limit){
                value = limit;
        } else if (value < -limit) {
                value = -limit;
        }
    }
    return value;
}


class WheelController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &nh)
    {
        std::string steer_joint_name;
        if (!nh.getParam("steer_joint", steer_joint_name)){
            ROS_ERROR("Parameter 'steer_joint' not set");
            return false;
        }
        std::string drive_joint_name;
        if (!nh.getParam("drive_joint", drive_joint_name)){
            ROS_ERROR("Parameter 'drive_joint' not set");
            return false;
        }
        steer_joint_ = hw->getHandle(steer_joint_name);
        drive_joint_ = hw->getHandle(drive_joint_name);


        nh.param("max_trans_velocity", max_vel_trans_, 0.0);
        if(max_vel_trans_ < 0){
            ROS_ERROR_STREAM("max_trans_velocity must be non-negative.");
            return false;
        }
        nh.param("max_rot_velocity", max_vel_rot_, 0.0);
        if(max_vel_rot_ < 0){
            ROS_ERROR_STREAM("max_rot_velocity must be non-negative.");
            return false;
        }
        double timeout;
        nh.param("timeout", timeout, 1.0);
        if(timeout < 0){
            ROS_ERROR_STREAM("timeout must be non-negative.");
            return false;
        }
        timeout_.fromSec(timeout);

        pub_divider_ =  nh.param("pub_divider", 0);
        twist_subscriber_ = nh.subscribe("command", 1, &WheelController::topicCallbackTwistCmd, this);
        commands_pub_.reset(new realtime_tools::RealtimePublisher<cob_base_controller_utils::WheelCommands>(nh, "wheel_commands", 5));

        commands_pub_->msg_.drive_target_velocity.resize(1);
        commands_pub_->msg_.steer_target_velocity.resize(1);
        commands_pub_->msg_.steer_target_position.resize(1);
        commands_pub_->msg_.steer_target_error.resize(1);

        return true;
    }

    virtual void starting(const ros::Time& time){
        cycles_ = 0;
    }

    virtual void update(const ros::Time& time, const ros::Duration& period)
    {
        updateCommand();

        steer_joint_.setCommand(wheel_command_.steer_vel);
        drive_joint_.setCommand(wheel_command_.drive_vel);

        if(cycles_ < pub_divider_ && (++cycles_) == pub_divider_){
            if(commands_pub_->trylock()){
                ++(commands_pub_->msg_.header.seq);
                commands_pub_->msg_.header.stamp = time;

                commands_pub_->msg_.drive_target_velocity[0] = wheel_command_.drive_vel;
                commands_pub_->msg_.steer_target_velocity[0] = wheel_command_.steer_vel;
                commands_pub_->msg_.steer_target_position[0] = 0.0;
                commands_pub_->msg_.steer_target_error[0] = 0.0;
                commands_pub_->unlockAndPublish();
            }
            cycles_ = 0;
        }
    }

private:
    struct Target {
        PlatformState state;
        bool updated;
        ros::Time stamp;
    } target_;

    PlatformState platform_state_;
    WheelState wheel_state_;
    WheelState wheel_command_;
    hardware_interface::JointHandle steer_joint_;
    hardware_interface::JointHandle drive_joint_;


    boost::mutex mutex_;
    ros::Subscriber twist_subscriber_;

    boost::scoped_ptr<realtime_tools::RealtimePublisher<cob_base_controller_utils::WheelCommands> > commands_pub_;
    uint32_t cycles_;
    uint32_t pub_divider_;

    ros::Duration timeout_;
    double max_vel_trans_, max_vel_rot_;

    void topicCallbackTwistCmd(const geometry_msgs::Twist::ConstPtr& msg){
        if(this->isRunning()){
            boost::mutex::scoped_lock lock(mutex_);
            if(isnan(msg->linear.x) || isnan(msg->linear.y) || isnan(msg->angular.z)) {
                ROS_FATAL("Received NaN-value in Twist message. Reset target to zero.");
                target_.state = PlatformState();
            }else{
                target_.state.velX = limitValue(msg->linear.x, max_vel_trans_);
                target_.state.velY = limitValue(msg->linear.y, max_vel_trans_);
                target_.state.rotTheta = limitValue(msg->angular.z, max_vel_rot_);
            }
            target_.updated = true;
            target_.stamp = ros::Time::now();
        }
    }

    void updateCommand(){
        //get JointState from JointHandles
        wheel_state_.steer_pos = steer_joint_.getPosition();
        wheel_state_.steer_vel = steer_joint_.getVelocity();
        wheel_state_.drive_pos = drive_joint_.getPosition();
        wheel_state_.drive_vel = drive_joint_.getVelocity();

        ///ToDo:
        //calculate inverse kinematics
        wheel_command_.steer_vel = target_.state.velX;
        wheel_command_.drive_vel = target_.state.rotTheta;
    }
};

}

PLUGINLIB_EXPORT_CLASS( cob_tricycle_controller::WheelController, controller_interface::ControllerBase)
