#include "GeomController.h"
#include "WheelControllerBase.h"

namespace cob_omni_drive_controller
{

class WheelController : public WheelControllerBase< GeomController<hardware_interface::VelocityJointInterface, UndercarriageCtrl> >
{
public:
    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){

        if(!GeomController::init(hw, controller_nh)) return false;

        return setup(root_nh,controller_nh);
    }
    virtual void update(const ros::Time& time, const ros::Duration& period){

        updateState();

        updateCtrl(time, period);

        for (unsigned i=0; i<wheel_commands_.size(); i++){
            steer_joints_[i].setCommand(wheel_commands_[i].dVelGearSteerRadS);
            drive_joints_[i].setCommand(wheel_commands_[i].dVelGearDriveRadS);
        }

    }

};

}

PLUGINLIB_EXPORT_CLASS( cob_omni_drive_controller::WheelController, controller_interface::ControllerBase)
