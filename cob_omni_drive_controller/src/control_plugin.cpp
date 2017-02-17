
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>

#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>
#include <geometry_msgs/Twist.h>

#include <boost/scoped_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include "GeomController.h"

#include <realtime_tools/realtime_publisher.h>
#include <cob_omni_drive_controller/WheelCommands.h>

#include <dynamic_reconfigure/server.h>
#include <cob_omni_drive_controller/SteerCtrlConfig.h>

namespace cob_omni_drive_controller
{

class WheelController: public GeomController<hardware_interface::VelocityJointInterface, UndercarriageCtrl>
{
public:
    WheelController() {}

    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){

        wheel_params_t wheel_params;
        if(!parseWheelParams(wheel_params, controller_nh) || !GeomController::init(hw, wheel_params)) return false;

        controller_nh.param("max_rot_velocity", max_vel_rot_, 0.0);
        if(max_vel_rot_ < 0){
            ROS_ERROR_STREAM("max_rot_velocity must be non-negative.");
            return false;
        }
        controller_nh.param("max_trans_velocity", max_vel_trans_, 0.0);
        if(max_vel_trans_ < 0){
            ROS_ERROR_STREAM("max_trans_velocity must be non-negative.");
            return false;
        }
        double timeout;
        controller_nh.param("timeout", timeout, 1.0);
        if(timeout < 0){
            ROS_ERROR_STREAM("timeout must be non-negative.");
            return false;
        }
        timeout_.fromSec(timeout);

        pub_divider_ =  controller_nh.param("pub_divider",0);

        wheel_commands_.resize(wheel_states_.size());
        twist_subscriber_ = controller_nh.subscribe("command", 1, &WheelController::topicCallbackTwistCmd, this);

        commands_pub_.reset(new realtime_tools::RealtimePublisher<cob_omni_drive_controller::WheelCommands>(controller_nh, "wheel_commands", 5));
       
        commands_pub_->msg_.drive_target_velocity.resize(wheel_states_.size());
        commands_pub_->msg_.steer_target_velocity.resize(wheel_states_.size());
        commands_pub_->msg_.steer_target_position.resize(wheel_states_.size());
        commands_pub_->msg_.steer_target_error.resize(wheel_states_.size());

        pos_ctrl_.init(wheel_params, controller_nh);

        return true;
    }
    virtual void starting(const ros::Time& time){
        geom_->reset();
        target_.updated = false;
        cycles_ = 0;
    }
    virtual void update(const ros::Time& time, const ros::Duration& period){

        GeomController::update();
        pos_ctrl_.try_configure(*geom_);

        {
            boost::mutex::scoped_try_lock lock(mutex_);
            if(lock){
                Target target = target_;
                target_.updated = false;

                if(!target.stamp.isZero() && !timeout_.isZero() && (time - target.stamp) > timeout_){
                    target_.stamp = ros::Time(); // only reset once
                    target.state  = UndercarriageCtrl::PlatformState();
                    target.updated = true;
                }
                lock.unlock();

                if(target.updated){
                   geom_->setTarget(target.state);
                }
            }
        }

        geom_->calcControlStep(wheel_commands_, period.toSec(), false);

        for (unsigned i=0; i<wheel_commands_.size(); i++){
            steer_joints_[i].setCommand(wheel_commands_[i].dVelGearSteerRadS);
            drive_joints_[i].setCommand(wheel_commands_[i].dVelGearDriveRadS);
        }
        
        if(cycles_ < pub_divider_ && (++cycles_) == pub_divider_){
            if(commands_pub_->trylock()){
                ++(commands_pub_->msg_.header.seq);
                commands_pub_->msg_.header.stamp = time;

                for (unsigned i=0; i<wheel_commands_.size(); i++){
                    commands_pub_->msg_.drive_target_velocity[i] = wheel_commands_[i].dVelGearDriveRadS;
                    commands_pub_->msg_.steer_target_velocity[i] = wheel_commands_[i].dVelGearSteerRadS;
                    commands_pub_->msg_.steer_target_position[i] = wheel_commands_[i].dAngGearSteerRad;
                    commands_pub_->msg_.steer_target_error[i] = wheel_commands_[i].dAngGearSteerRadDelta;
                }
                commands_pub_->unlockAndPublish();
            
            }
            cycles_ = 0;
        }
    }
    virtual void stopping(const ros::Time& time) {}

private:
    struct Target {
        UndercarriageCtrl::PlatformState state;
        bool updated;
        ros::Time stamp;
    } target_;

    class PosCtrl {
    public:
        PosCtrl() : updated(false) {}
        void try_configure(UndercarriageCtrl &ctrl){
            boost::recursive_mutex::scoped_try_lock lock(mutex);
            if(lock && updated){
                ctrl.configure(pos_ctrl_params);
                updated = false;
            }
        }
        void init(const wheel_params_t &params, const ros::NodeHandle &nh){
            boost::recursive_mutex::scoped_lock lock(mutex);
            pos_ctrl_params.resize(params.size());
            reconfigure_server_axes_.clear();

            reconfigure_server_.reset( new dynamic_reconfigure::Server<SteerCtrlConfig>(mutex,ros::NodeHandle(nh, "default/steer_ctrl")));
            reconfigure_server_->setCallback(boost::bind(&PosCtrl::setForAll, this, _1, _2)); // this writes the default into pos_ctrl_params
            {
                SteerCtrlConfig config;
                copy(config, params.front().ctrl.pos_ctrl);
                reconfigure_server_->setConfigDefault(config);
            }

            for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                boost::shared_ptr<dynamic_reconfigure::Server<SteerCtrlConfig> > dr(new dynamic_reconfigure::Server<SteerCtrlConfig>(mutex, ros::NodeHandle(nh, params[i].geom.steer_name)));
                for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                    SteerCtrlConfig config;
                    copy(config, params[i].ctrl.pos_ctrl);
                    dr->setConfigDefault(config);
                    dr->updateConfig(config);
                    dr->setCallback(boost::bind(&PosCtrl::setForOne, this, i, _1, _2));  // this writes the joint-specific config into pos_ctrl_params
                    reconfigure_server_axes_.push_back(dr);
                }
            }
        }
    private:
        static void copy(UndercarriageCtrl::PosCtrlParams &params, const SteerCtrlConfig &config){
            params.dSpring = config.spring;
            params.dDamp = config.damp;
            params.dVirtM = config.virt_mass;
            params.dDPhiMax = config.d_phi_max;
            params.dDDPhiMax = config.dd_phi_max;
        }
        static void copy(SteerCtrlConfig &config, const UndercarriageCtrl::PosCtrlParams &params){
            config.spring = params.dSpring;
            config.damp = params.dDamp;
            config.virt_mass = params.dVirtM;
            config.d_phi_max = params.dDPhiMax;
            config.dd_phi_max = params.dDDPhiMax;
        }
        // these function don't get locked
        void setForAll(SteerCtrlConfig &config, uint32_t /*level*/) {
            for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                copy(pos_ctrl_params[i], config);
                reconfigure_server_axes_[i]->updateConfig(config);
            }
            updated = true;
        }
        void setForOne(size_t i, SteerCtrlConfig &config, uint32_t /*level*/) {
            copy(pos_ctrl_params[i], config);
            updated = true;
        }
        std::vector<UndercarriageCtrl::PosCtrlParams> pos_ctrl_params;
        boost::recursive_mutex mutex; // dynamic_reconfigure::Server calls the callback from the setCallback function
        bool updated;
        boost::scoped_ptr< dynamic_reconfigure::Server<SteerCtrlConfig> > reconfigure_server_;
        std::vector<boost::shared_ptr< dynamic_reconfigure::Server<SteerCtrlConfig> > > reconfigure_server_axes_; // TODO: use unique_ptr
    } pos_ctrl_;

    std::vector<UndercarriageCtrl::WheelCommand> wheel_commands_;

    boost::mutex mutex_;
    ros::Subscriber twist_subscriber_;
    
    boost::scoped_ptr<realtime_tools::RealtimePublisher<cob_omni_drive_controller::WheelCommands> > commands_pub_;
    uint32_t cycles_;
    uint32_t pub_divider_;
    
    ros::Duration timeout_;
    double max_vel_trans_, max_vel_rot_;

    void topicCallbackTwistCmd(const geometry_msgs::Twist::ConstPtr& msg){
        if(isRunning()){
            boost::mutex::scoped_lock lock(mutex_);
            if(isnan(msg->linear.x) || isnan(msg->linear.y) || isnan(msg->angular.z)) {
                ROS_FATAL("Received NaN-value in Twist message. Reset target to zero.");
                target_.state = UndercarriageCtrl::PlatformState();
            }else{
                target_.state.setVelX(UndercarriageCtrl::limitValue(msg->linear.x, max_vel_trans_));
                target_.state.setVelY(UndercarriageCtrl::limitValue(msg->linear.y, max_vel_trans_));
                target_.state.dRotRobRadS = UndercarriageCtrl::limitValue(msg->angular.z, max_vel_rot_);
            }
            target_.updated = true;
            target_.stamp = ros::Time::now();
        }
    }
};

}

PLUGINLIB_EXPORT_CLASS( cob_omni_drive_controller::WheelController, controller_interface::ControllerBase)
