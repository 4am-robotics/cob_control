#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class CobHWInterfaceVel : public hardware_interface::RobotHW
{
public:
  CobHWInterfaceVel();

  void read();
  void write();

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  
  std::vector<std::string> joint_names;
  
  std::vector<double> cmd;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;
};
