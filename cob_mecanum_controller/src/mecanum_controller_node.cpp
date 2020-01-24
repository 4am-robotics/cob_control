#include "cob_mecanum_controller/mecanum_controller.h"
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

    std::string static_frame = "map";
    pnh.getParam("static_frame", static_frame);
    std::string odom_frame = "map";
    pnh.getParam("odom_frame", odom_frame);

    controller_ = std::make_shared<cob_mecanum_controller::MecanumController>(lx, ly, r);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10, false);
    joint_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("wheel_command", 10, false);

    twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, [this](auto const& msg) {
      Eigen::Vector3d twist;
      twist << msg->linear.x, msg->linear.y, msg->angular.z;
      auto wheel_velocities = controller_->twistToWheel(twist);
      sensor_msgs::JointState joint_command;
      joint_command.velocity = std::vector<double>(
          wheel_velocities.data(), wheel_velocities.data() + wheel_velocities.rows() * wheel_velocities.cols());
      joint_cmd_pub_.publish(joint_command);
    });

    joint_state_sub_ =
        nh_.subscribe<sensor_msgs::JointState>("wheel_state", 1, [this, static_frame, odom_frame](auto const& msg) {
          Eigen::Vector4d wheel_velocities(msg->velocity.data());
          auto twist = controller_->wheelToTwist(wheel_velocities);
          nav_msgs::Odometry odom_msg;
          odom_msg.header.frame_id = static_frame;
          odom_msg.child_frame_id = odom_frame;

          odom_msg.twist.twist.linear.x = twist.x();
          odom_msg.twist.twist.linear.y = twist.y();
          odom_msg.twist.twist.angular.z = twist.z();
          odom_pub_.publish(odom_msg);
        });
  }

protected:
  ros::NodeHandle nh_;
  ros::Subscriber twist_sub_;
  ros::Subscriber joint_state_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher joint_cmd_pub_;
  std::shared_ptr<cob_mecanum_controller::MecanumController> controller_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "mecanum_controller");
  MecanumControllerNode mcn;
  ros::spin();
}
