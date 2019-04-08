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
#include <angles/angles.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>

#include <cob_tricycle_controller/param_parser.h>
#include <cob_base_controller_utils/WheelCommands.h>
#include <geometry_msgs/Twist.h>

#include <cob_tricycle_controller/TricycleCtrlTypes.h>

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


class WheelController : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface, hardware_interface::PositionJointInterface>
{
public:
    virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &nh)
    {
        if (!nh.getParam("steer_joint", wheel_state_.steer_name)){
            ROS_ERROR("Parameter 'steer_joint' not set");
            return false;
        }
        if (!nh.getParam("drive_joint", wheel_state_.drive_name)){
            ROS_ERROR("Parameter 'drive_joint' not set");
            return false;
        }

        hardware_interface::VelocityJointInterface* v = robot_hw->get<hardware_interface::VelocityJointInterface>();
        hardware_interface::PositionJointInterface* p = robot_hw->get<hardware_interface::PositionJointInterface>();
        steer_joint_ = p->getHandle(wheel_state_.steer_name);
        drive_joint_ = v->getHandle(wheel_state_.drive_name);

        urdf::Model model;
        std::string description_name;
        bool has_model = nh.searchParam("robot_description", description_name) &&  model.initParam(description_name);

        urdf::Vector3 steer_pos;
        urdf::JointConstSharedPtr steer_joint;
        urdf::JointConstSharedPtr drive_joint;

        if(has_model){
            steer_joint = model.getJoint(wheel_state_.steer_name);
            if(steer_joint){
                tf2::Transform transform;
                //root link for tricycle is "base_pivot_link"
                if(parseWheelTransform(wheel_state_.steer_name, model.getRoot()->name, transform, &model)){
                    wheel_state_.pos_x = transform.getOrigin().getX();
                    wheel_state_.pos_y = transform.getOrigin().getY();
                    wheel_state_.radius = transform.getOrigin().getZ();
                    wheel_state_.sign = cos(transform.getRotation().getAngle());
                }
                nh.param("max_steer_rate", max_steer_rate_, 0.0);
                max_steer_rate_ = (max_steer_rate_ != 0.0 && max_steer_rate_ < steer_joint->limits->velocity) ? max_steer_rate_ : steer_joint->limits->velocity;
            }
            drive_joint = model.getJoint(wheel_state_.drive_name);
            if(drive_joint){
                nh.param("max_drive_rate", max_drive_rate_, 0.0);
                max_drive_rate_ = (max_drive_rate_ != 0.0 && max_drive_rate_ < drive_joint->limits->velocity) ? max_drive_rate_ : drive_joint->limits->velocity;
            }
        }

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
        {
            boost::mutex::scoped_try_lock lock(mutex_);
            if(lock){
                Target target = target_;
                target_.updated = false;

                if(!target.stamp.isZero() && !timeout_.isZero() && (time - target.stamp) > timeout_){
                    ROS_WARN_STREAM("target timed out");
                    target_.stamp = ros::Time(); // only reset once
                    target.state  = PlatformState();
                    target.updated = true;
                }
                lock.unlock();
            }
        }

        updateCommand();

        steer_joint_.setCommand(wheel_command_.steer_pos);
        drive_joint_.setCommand(wheel_command_.drive_vel);

        if(cycles_ < pub_divider_ && (++cycles_) == pub_divider_){
            if(commands_pub_->trylock()){
                ++(commands_pub_->msg_.header.seq);
                commands_pub_->msg_.header.stamp = time;

                commands_pub_->msg_.drive_target_velocity[0] = wheel_command_.drive_vel;
                commands_pub_->msg_.steer_target_velocity[0] = 0.0;
                commands_pub_->msg_.steer_target_position[0] = wheel_command_.steer_pos;
                commands_pub_->msg_.steer_target_error[0] = wheel_command_.steer_pos - wheel_state_.steer_pos;
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
    double max_steer_rate_, max_drive_rate_;

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

        // calculate inverse kinematics
        // http://www.wolframalpha.com/input/?i=Solve%5Bx%3D%3Dw*cos(a),+phi%3D%3Dw*sin(a)%2Fr,a,w%5D
        double r_base = wheel_state_.pos_x * wheel_state_.sign;
        double k = sqrt(pow(r_base,2)*pow(target_.state.rotTheta,2) + pow(target_.state.velX,2));
        if (target_.state.rotTheta != 0)
        {
            double a1 = 2 * atan2(k - target_.state.velX, r_base*target_.state.rotTheta);
            a1 = angles::normalize_angle(a1);
            double b1 = k / wheel_state_.radius;

            double a2 = -2 * atan2(k + target_.state.velX, r_base*target_.state.rotTheta);
            a2 = angles::normalize_angle(a2);
            double b2 = - k / wheel_state_.radius;

            if (fabs(a1-wheel_state_.steer_pos) <= fabs(a2-wheel_state_.steer_pos))
            {
                wheel_command_.steer_pos = a1;
                wheel_command_.drive_vel = b1;
            }
            else
            {
                wheel_command_.steer_pos = a2;
                wheel_command_.drive_vel = b2;
            }
        }
        else if (target_.state.velX != 0)
        {
            if (fabs(0.0-wheel_state_.steer_pos) <= fabs(M_PI-wheel_state_.steer_pos))
            {
                wheel_command_.steer_pos = 0.0;
                wheel_command_.drive_vel = target_.state.velX / wheel_state_.radius;
            }
            else
            {
                wheel_command_.steer_pos = M_PI;
                wheel_command_.drive_vel = -target_.state.velX / wheel_state_.radius;
            }
        }
        else {
            wheel_command_.steer_pos = wheel_state_.steer_pos;
            wheel_command_.drive_vel = 0.0;
        }

        wheel_command_.drive_vel = limitValue(wheel_command_.drive_vel, max_drive_rate_);
    }
};

}

PLUGINLIB_EXPORT_CLASS( cob_tricycle_controller::WheelController, controller_interface::ControllerBase)
