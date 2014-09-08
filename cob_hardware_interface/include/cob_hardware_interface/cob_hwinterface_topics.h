#include "ros/ros.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointVelocities.h>

#include <boost/thread.hpp>

class CobHWInterfaceTopics : public hardware_interface::RobotHW
{
  public:
    CobHWInterfaceTopics();
    void read();
    void write();
    void jointstates_callback(const sensor_msgs::JointState::ConstPtr& msg);

  private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    
    ros::NodeHandle nh;
    std::vector<std::string> joint_names;
    
    ros::Publisher cmd_vel_pub;
    ros::Subscriber jointstates_sub;


    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> eff;
    std::vector<double> cmd;
    boost::mutex mtx_;
};
