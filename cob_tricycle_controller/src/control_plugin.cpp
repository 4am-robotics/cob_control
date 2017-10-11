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
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <cob_tricycle_controller/TricycleCtrlTypes.h>

#include <pluginlib/class_list_macros.h>

namespace cob_tricycle_controller
{

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
    }
    virtual void update(const ros::Time& time, const ros::Duration& period)
    {
        steer_joint_.setCommand(0.0);
        drive_joint_.setCommand(0.1);
    }

private:
    PlatformState platform_state_;
    WheelState wheel_state_;
    hardware_interface::JointHandle steer_joint_;
    hardware_interface::JointHandle drive_joint_;
};

}

PLUGINLIB_EXPORT_CLASS( cob_tricycle_controller::WheelController, controller_interface::ControllerBase)
