
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>

#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>
#include <geometry_msgs/Twist.h>

#include <boost/scoped_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include "GeomController.h"

namespace cob_omni_drive_controller
{

class WheelController: public GeomController<hardware_interface::VelocityJointInterface>
{
public:
    WheelController() {}

    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){

        if(!GeomController::init(hw, controller_nh)) return true;

        wheel_commands_.resize(wheel_states_.size());
        twist_subscriber_ = controller_nh.subscribe("command", 1, &WheelController::topicCallbackTwistCmd, this);
        

        return true;
  }
    virtual void starting(const ros::Time& time){
        geom_->reset();
    }
    virtual void update(const ros::Time& time, const ros::Duration& period){

        GeomController::update();

        boost::shared_ptr<const geometry_msgs::Twist> target;
        {
            boost::mutex::scoped_try_lock lock(mutex_);
            if(lock) target = target_;
        }
        if(target){
            target_.reset();
            platform_state_.dVelLongMMS = target->linear.x * 1000.0;
            platform_state_.dVelLatMMS = target->linear.y * 1000.0;
            platform_state_.dRotRobRadS = target->angular.z;
            geom_->setTarget(platform_state_);
        }

        geom_->calcControlStep(wheel_commands_, period.toSec(), false);

        for (unsigned i=0; i<wheel_commands_.size(); i++){
            steer_joints_[i].setCommand(wheel_commands_[i].dVelGearSteerRadS);
            drive_joints_[i].setCommand(wheel_commands_[i].dVelGearDriveRadS);
        }
        
    }
    virtual void stopping(const ros::Time& time) {}

private:
    std::vector<UndercarriageCtrlGeom::WheelState> wheel_commands_;
    UndercarriageCtrlGeom::PlatformState platform_state_;

    ros::Subscriber twist_subscriber_;
    geometry_msgs::Twist::ConstPtr target_;
    boost::mutex mutex_;
  
    void topicCallbackTwistCmd(const geometry_msgs::Twist::ConstPtr& msg){
        if(isRunning()){
            boost::mutex::scoped_lock lock(mutex_);
            target_ = msg;
        }
    }


};

}

PLUGINLIB_EXPORT_CLASS( cob_omni_drive_controller::WheelController, controller_interface::ControllerBase)
