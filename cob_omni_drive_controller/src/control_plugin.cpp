
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>

#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>
#include <geometry_msgs/Twist.h>

#include <boost/scoped_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>

namespace cob_omni_drive_controller
{

template<typename T> class GeomController: public controller_interface::Controller<T> {
protected:
    std::vector<hardware_interface::JointHandle> steer_joints_, drive_joints_;
    std::vector<UndercarriageCtrlGeom::WheelState> wheel_states_;
    boost::scoped_ptr<UndercarriageCtrlGeom> geom_;
public:    
    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){

        std::vector<std::string> steer_names, drive_names;

        if (!controller_nh.getParam("steer_joints", steer_names)){
            ROS_ERROR("Parameter 'steer_joints' not set");
            return false;
        }
        if (!controller_nh.getParam("drives_joints", drive_names)){
            ROS_ERROR("Parameter 'drives_joints' not set");
            return false;
        }

        if (steer_names.size()!=drive_names.size()){
            ROS_ERROR("Number of steer joints does not match number of drive joints");
            return false;
        }

        if (drive_names.size() < 3){
            ROS_ERROR("At least three wheel are needed.");
            return false;
        }

        for (unsigned i=0; i<steer_names.size(); i++){
            steer_joints_.push_back(hw->getHandle(steer_names[i]));
            drive_joints_.push_back(hw->getHandle(drive_names[i]));
        }
        wheel_states_.resize(steer_names.size());

        std::vector<UndercarriageCtrlGeom::WheelParams> params;

        std::string ini_directory;
        if (!controller_nh.getParam("ini_directory", ini_directory)){
            ROS_ERROR("Parameter 'ini_directory' not set");
            return false;
        }

        try{
            UndercarriageCtrlGeom::parseIniFiles(params, ini_directory);
        }
        catch(...){
            ROS_ERROR("INI file parsing failed");
            return false;
        }
        geom_.reset(new UndercarriageCtrlGeom(params));
        return true;
    }

    virtual void update(const ros::Time& time, const ros::Duration& period){

        for (unsigned i=0; i<wheel_states_.size(); i++){
            wheel_states_[i].dAngGearSteerRad = steer_joints_[i].getPosition();
            wheel_states_[i].dVelGearSteerRadS = steer_joints_[i].getVelocity();
            wheel_states_[i].dVelGearDriveRadS = drive_joints_[i].getVelocity();
        }
        geom_->updateWheelStates(wheel_states_);
    }    
};

// this controller gets access to the JointStateInterface
class WheelController: public GeomController<hardware_interface::VelocityJointInterface>
{
public:
    WheelController() {}

    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){

        if(!GeomController::init(hw,root_nh, controller_nh)) return true;

        wheel_commands_.resize(wheel_states_.size());
        twist_subscriber_ = controller_nh.subscribe("command", 1, &WheelController::topicCallbackTwistCmd, this);
        

        return true;
  }
    virtual void starting(const ros::Time& time){
        geom_->reset();
    }
    virtual void update(const ros::Time& time, const ros::Duration& period){

        GeomController::update(time, period);

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
