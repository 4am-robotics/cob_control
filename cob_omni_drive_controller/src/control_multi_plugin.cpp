#include "GeomController.h"
#include "WheelControllerBase.h"
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace cob_omni_drive_controller
{

class GeomMultiController : public GeomControllerBase< hardware_interface::JointHandle, UndercarriageDirectCtrl>,
    public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface, hardware_interface::PositionJointInterface> {
};

class WheelMultiController : public WheelControllerBase< GeomMultiController >
{
public:
    virtual bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){
        std::vector<UndercarriageDirectCtrl::WheelParams> wheel_params;
        if(!parseWheelParams(wheel_params, controller_nh)) return false;
        if(!GeomControllerBase::setup(wheel_params)) return false;

        hardware_interface::VelocityJointInterface* v = robot_hw->get<hardware_interface::VelocityJointInterface>();
        hardware_interface::PositionJointInterface* p = robot_hw->get<hardware_interface::PositionJointInterface>();

        try{
            for (unsigned i=0; i<wheel_params.size(); i++){
                this->steer_joints_.push_back(p->getHandle(wheel_params[i].geom.steer_name));
                this->drive_joints_.push_back(v->getHandle(wheel_params[i].geom.drive_name));
            }
        }
        catch(const std::exception &e){
            ROS_ERROR_STREAM("Error while attaching handles: " << e.what());
            return false;
        }
        return this->setup(root_nh,controller_nh);
    }
    virtual void update(const ros::Time& time, const ros::Duration& period){

        updateState();

        updateCtrl(time, period);

        for (unsigned i=0; i<wheel_commands_.size(); i++){
            steer_joints_[i].setCommand(wheel_commands_[i].dAngGearSteerRad);
            drive_joints_[i].setCommand(wheel_commands_[i].dVelGearDriveRadS);
        }

    }
};

}

PLUGINLIB_EXPORT_CLASS( cob_omni_drive_controller::WheelMultiController, controller_interface::ControllerBase)
