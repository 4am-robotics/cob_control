#include "ros/ros.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "control_msgs/JointTrajectoryControllerState.h"
#include "brics_actuator/JointVelocities.h"
#include "brics_actuator/JointValue.h"


class CobHWInterfaceVel : public hardware_interface::RobotHW
{
    public:
        CobHWInterfaceVel();
        void read();
        void write();
        void state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;
        std::vector<std::string> joint_names;        
        std::vector<double> cmd;
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> eff;
        std::vector<double> cmd_temp;
        std::vector<double> pos_temp;
        std::vector<double> vel_temp;
        std::vector<double> eff_temp;          
};
