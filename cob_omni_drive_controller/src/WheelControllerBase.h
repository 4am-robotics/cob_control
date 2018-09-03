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


#ifndef H_WHEEL_CONTROLLER_IMPL
#define H_WHEEL_CONTROLLER_IMPL

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>

#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>
#include <geometry_msgs/Twist.h>

#include <boost/scoped_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <realtime_tools/realtime_publisher.h>
#include <cob_base_controller_utils/WheelCommands.h>

namespace cob_omni_drive_controller
{

template<typename T> class WheelControllerBase: public T
{
public:
    bool setup(ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){
        controller_nh.param("max_rot_velocity", max_vel_rot_, 0.0);
        if(max_vel_rot_ < 0){
            ROS_ERROR_STREAM("max_rot_velocity must be non-negative.");
            return false;
        }
        controller_nh.param("max_trans_velocity", max_vel_trans_, 0.0);
        if(max_vel_trans_ < 0){
            ROS_ERROR_STREAM("max_trans_velocity must be non-negative.");
            return false;
        }
        double timeout;
        controller_nh.param("timeout", timeout, 1.0);
        if(timeout < 0){
            ROS_ERROR_STREAM("timeout must be non-negative.");
            return false;
        }
        timeout_.fromSec(timeout);
        
        pub_divider_ =  controller_nh.param("pub_divider",0);

        wheel_commands_.resize(this->wheel_states_.size());
        twist_subscriber_ = controller_nh.subscribe("command", 1, &WheelControllerBase::topicCallbackTwistCmd, this);

        commands_pub_.reset(new realtime_tools::RealtimePublisher<cob_base_controller_utils::WheelCommands>(controller_nh, "wheel_commands", 5));
       
        commands_pub_->msg_.drive_target_velocity.resize(this->wheel_states_.size());
        commands_pub_->msg_.steer_target_velocity.resize(this->wheel_states_.size());
        commands_pub_->msg_.steer_target_position.resize(this->wheel_states_.size());
        commands_pub_->msg_.steer_target_error.resize(this->wheel_states_.size());

        return true;
  }
    virtual void starting(const ros::Time& time){
        this->geom_->reset();
        target_.updated = false;
        cycles_ = 0;
    }
    void updateCtrl(const ros::Time& time, const ros::Duration& period){
        {
            boost::mutex::scoped_try_lock lock(mutex_);
            if(lock){
                Target target = target_;
                target_.updated = false;

                if(!target.stamp.isZero() && !timeout_.isZero() && (time - target.stamp) > timeout_){
                    target_.stamp = ros::Time(); // only reset once
                    target.state  = PlatformState();
                    target.updated = true;
                }
                lock.unlock();

                if(target.updated){
                   this->geom_->setTarget(target.state);
                }
            }
        }

        this->geom_->calcControlStep(wheel_commands_, period.toSec(), false);

        if(cycles_ < pub_divider_ && (++cycles_) == pub_divider_){
            if(commands_pub_->trylock()){
                ++(commands_pub_->msg_.header.seq);
                commands_pub_->msg_.header.stamp = time;

                for (unsigned i=0; i<wheel_commands_.size(); i++){
                    commands_pub_->msg_.drive_target_velocity[i] = wheel_commands_[i].dVelGearDriveRadS;
                    commands_pub_->msg_.steer_target_velocity[i] = wheel_commands_[i].dVelGearSteerRadS;
                    commands_pub_->msg_.steer_target_position[i] = wheel_commands_[i].dAngGearSteerRad;
                    commands_pub_->msg_.steer_target_error[i] = wheel_commands_[i].dAngGearSteerRadDelta;
                }
                commands_pub_->unlockAndPublish();
            
            }
            cycles_ = 0;
        }
    }
    virtual void stopping(const ros::Time& time) {}

protected:
    struct Target {
        PlatformState state;
        bool updated;
        ros::Time stamp;
    } target_;

    std::vector<WheelCommand> wheel_commands_;

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
                target_.state.setVelX(limitValue(msg->linear.x, max_vel_trans_));
                target_.state.setVelY(limitValue(msg->linear.y, max_vel_trans_));
                target_.state.dRotRobRadS = limitValue(msg->angular.z, max_vel_rot_);
            }
            target_.updated = true;
            target_.stamp = ros::Time::now();
        }
    }


};

}
#endif
