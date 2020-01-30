/*
 * Copyright 2020 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
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

#include <cob_mecanum_controller/mecanum_controller.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

#include <exception>
class MecanumControllerNode
{
public:
  MecanumControllerNode() : nh_()
  {
    double lx, ly, r;
    bool all_parameters_set = true;
    ros::NodeHandle pnh("~");
    if (!pnh.getParam("lx", lx))
    {
      ROS_ERROR_STREAM("Parameter lx was not declared in the scope");
      all_parameters_set = false;
    }
    if (!pnh.getParam("ly", ly))
    {
      ROS_ERROR_STREAM("Parameter ly was not declared in the scope");
      all_parameters_set = false;
    }
    if (!pnh.getParam("r", r))
    {
      ROS_ERROR_STREAM("Parameter r was not declared in the scope");
      all_parameters_set = false;
    }

    if (!all_parameters_set)
    {
      throw std::runtime_error("At least one parameter is missing.");
    }

    static_frame_ = "map";
    pnh.getParam("static_frame", static_frame_);
    odom_frame_ = "map";
    pnh.getParam("odom_frame", odom_frame_);

    controller_ = std::make_shared<cob_mecanum_controller::MecanumController>(lx, ly, r);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10, false);
    joint_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("wheel_command", 10, false);

    twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &MecanumControllerNode::twistCallback, this);

    joint_state_sub_ =
        nh_.subscribe<sensor_msgs::JointState>("wheel_state", 1, &MecanumControllerNode::jointStateCallback, this);
  }

protected:
  ros::NodeHandle nh_;
  ros::Subscriber twist_sub_;
  ros::Subscriber joint_state_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher joint_cmd_pub_;
  std::shared_ptr<cob_mecanum_controller::MecanumController> controller_;

  std::string static_frame_;
  std::string odom_frame_;

  void twistCallback(const geometry_msgs::Twist msg)
  {
    Eigen::Vector3d twist;
    twist << msg.linear.x, msg.linear.y, msg.angular.z;
    Eigen::Vector4d wheel_velocities = controller_->twistToWheel(twist);
    sensor_msgs::JointState joint_command;
    joint_command.velocity = std::vector<double>(
        wheel_velocities.data(), wheel_velocities.data() + wheel_velocities.rows() * wheel_velocities.cols());
    joint_cmd_pub_.publish(joint_command);
  }

  void jointStateCallback(const sensor_msgs::JointState msg)
  {
    Eigen::Vector4d wheel_velocities(msg.velocity.data());
    Eigen::Vector3d twist = controller_->wheelToTwist(wheel_velocities);
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = static_frame_;
    odom_msg.child_frame_id = odom_frame_;

    odom_msg.twist.twist.linear.x = twist.x();
    odom_msg.twist.twist.linear.y = twist.y();
    odom_msg.twist.twist.angular.z = twist.z();
    odom_pub_.publish(odom_msg);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mecanum_controller");
  MecanumControllerNode mcn;
  ros::spin();
}
