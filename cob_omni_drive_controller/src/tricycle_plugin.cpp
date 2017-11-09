/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
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

#include <controller_interface/controller.h>
#include <boost/scoped_ptr.hpp>

#include "WheelControllerBase.h"

#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>
#include <cob_omni_drive_controller/OdometryTracker.h>

#include <dynamic_reconfigure/server.h>
#include <cob_omni_drive_controller/SteerCtrlConfig.h>

#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>

#include <pluginlib/class_list_macros.h>

namespace cob_tricycle_controller
{

bool parseWheelParams(std::vector<UndercarriageGeom::WheelParams> &params, const ros::NodeHandle &nh, bool read_urdf = true);

template<typename Interface> class TricycleControllerBase : public controller_interface::Controller<Interface> {
    typename Interface::ResourceHandleType steer_back_, drive_back_;
    std::vector<WheelState> wheel_states_;
protected:
    boost::scoped_ptr<UndercarriageGeom> geom_;
public:
    void updateState(){

        wheel_states_[1].dAngGearSteerRad = steer_back_.getPosition();
        wheel_states_[1].dVelGearSteerRadS = steer_back_.getVelocity();
        wheel_states_[1].dVelGearDriveRadS = drive_back_.getVelocity();

        wheel_states_[0].dVelGearDriveRadS = wheel_states_[2].dVelGearDriveRadS = cos(steer_back_.getPosition()) * drive_back_.getVelocity(); // TODO;

        geom_->updateWheelStates(wheel_states_);
    }

    bool init(Interface* hw, ros::NodeHandle& controller_nh){
        std::vector<UndercarriageGeom::WheelParams> wheel_params;
        if(!cob_tricycle_controller::parseWheelParams(wheel_params, controller_nh)) return false; // read from rosparam/URDF

        if (wheel_params.size() < 3){ // TODO
            ROS_ERROR("At least three wheel are needed.");
            return false;
        }
        wheel_states_.resize(wheel_params.size());
        geom_.reset(new UndercarriageGeom(wheel_params));
        try{
            steer_back_ = hw->getHandle("b_caster_rotation_joint");
            drive_back_ = hw->getHandle("b_caster_r_wheel_joint");
        }
        catch(const std::exception &e){
            ROS_ERROR_STREAM("Error while attaching handles: " << e.what());
            return false;
        }

        return true;
    }
};


//class WheelController : public cob_omni_drive_controller::WheelControllerBase< TricycleControllerBase<hardware_interface::VelocityJointInterface> >
//{
//public:
    //virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){

        //wheel_params_t wheel_params;
        //if(!cob_omni_drive_controller::parseWheelParams(wheel_params, controller_nh) || !TricycleGeomController::init(hw, wheel_params)) return false;

        //pos_ctrl_.init(wheel_params, controller_nh);

        //return setup(root_nh, controller_nh);
    //}
    //virtual void update(const ros::Time& time, const ros::Duration& period){

        //updateState();
        //pos_ctrl_.try_configure(*geom_);

        //updateCtrl(time, period);

        //////for (unsigned i=0; i<wheel_commands_.size(); i++){
            //////steer_joints_[i].setCommand(wheel_commands_[i].dVelGearSteerRadS);
            //////drive_joints_[i].setCommand(wheel_commands_[i].dVelGearDriveRadS);
        //////}
        //steer_joints_[0].setCommand(0.0);
        //steer_joints_[1].setCommand(wheel_commands_[1].dVelGearSteerRadS);
        //steer_joints_[2].setCommand(0.0);
        //drive_joints_[0].setCommand(cos(steer_joints_[1].getPosition()) * drive_joints_[1].getVelocity());
        //drive_joints_[1].setCommand(wheel_commands_[1].dVelGearDriveRadS);
        //drive_joints_[2].setCommand(cos(steer_joints_[1].getPosition()) * drive_joints_[1].getVelocity());
    //}

    //class PosCtrl {
    //public:
        //PosCtrl() : updated(false) {}
        //void try_configure(UndercarriageCtrl &ctrl){
            //boost::recursive_mutex::scoped_try_lock lock(mutex);
            //if(lock && updated){
                //ctrl.configure(pos_ctrl_params);
                //updated = false;
            //}
        //}
        //void init(const wheel_params_t &params, const ros::NodeHandle &nh){
            //boost::recursive_mutex::scoped_lock lock(mutex);
            //pos_ctrl_params.resize(params.size());
            //reconfigure_server_axes_.clear();

            //reconfigure_server_.reset( new dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig>(mutex,ros::NodeHandle(nh, "default/steer_ctrl")));
            //reconfigure_server_->setCallback(boost::bind(&PosCtrl::setForAll, this, _1, _2)); // this writes the default into pos_ctrl_params
            //{
                //cob_omni_drive_controller::SteerCtrlConfig config;
                //copy(config, params.front().pos_ctrl);
                //reconfigure_server_->setConfigDefault(config);
            //}

            //for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                //boost::shared_ptr<dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig> > dr(new dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig>(mutex, ros::NodeHandle(nh, params[i].geom.steer_name)));
                //cob_omni_drive_controller::SteerCtrlConfig config;
                //copy(config, params[i].pos_ctrl);
                //dr->setConfigDefault(config);
                //dr->updateConfig(config);
                //dr->setCallback(boost::bind(&PosCtrl::setForOne, this, i, _1, _2));  // this writes the joint-specific config into pos_ctrl_params
                //reconfigure_server_axes_.push_back(dr);
            //}
        //}
    //private:
        //static void copy(PosCtrlParams &params, const cob_omni_drive_controller::SteerCtrlConfig &config){
            //params.dSpring = config.spring;
            //params.dDamp = config.damp;
            //params.dVirtM = config.virt_mass;
            //params.dDPhiMax = config.d_phi_max;
            //params.dDDPhiMax = config.dd_phi_max;
        //}
        //static void copy(cob_omni_drive_controller::SteerCtrlConfig &config, const PosCtrlParams &params){
            //config.spring = params.dSpring;
            //config.damp = params.dDamp;
            //config.virt_mass = params.dVirtM;
            //config.d_phi_max = params.dDPhiMax;
            //config.dd_phi_max = params.dDDPhiMax;
        //}
        //// these function don't get locked
        //void setForAll(cob_omni_drive_controller::SteerCtrlConfig &config, uint32_t /*level*/) {
            //ROS_INFO("configure all steers: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
            //for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                //copy(pos_ctrl_params[i], config);
                //if(!reconfigure_server_axes_.empty()){
                    //reconfigure_server_axes_[i]->setConfigDefault(config);
                    //reconfigure_server_axes_[i]->updateConfig(config);
                //}
            //}
            //updated = true;
        //}
        //void setForOne(size_t i, cob_omni_drive_controller::SteerCtrlConfig &config, uint32_t /*level*/) {
            //ROS_INFO("configure steer %d: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", (int)i, config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
            //copy(pos_ctrl_params[i], config);
            //updated = true;
        //}
        //std::vector<PosCtrlParams> pos_ctrl_params;
        //boost::recursive_mutex mutex; // dynamic_reconfigure::Server calls the callback from the setCallback function
        //bool updated;
        //boost::scoped_ptr< dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig> > reconfigure_server_;
        //std::vector<boost::shared_ptr< dynamic_reconfigure::Server<cob_omni_drive_controller::SteerCtrlConfig> > > reconfigure_server_axes_; // TODO: use unique_ptr
    //} pos_ctrl_;
//};


class OdometryController: public TricycleControllerBase<hardware_interface::JointStateInterface>
{
public:
    OdometryController() {}

    virtual bool init(hardware_interface::JointStateInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){

        if(!TricycleControllerBase::init(hw, controller_nh)) return false;

        double publish_rate;
        if (!controller_nh.getParam("publish_rate", publish_rate)){
            ROS_ERROR("Parameter 'publish_rate' not set");
            return false;
        }
        if(publish_rate <= 0){
            ROS_ERROR_STREAM("publish_rate must be positive.");
            return false;
        }

        const std::string frame_id = controller_nh.param("frame_id", std::string("odom"));
        const std::string child_frame_id = controller_nh.param("child_frame_id", std::string("base_footprint"));
        const double cov_pose = controller_nh.param("cov_pose", 0.1);
        const double cov_twist = controller_nh.param("cov_twist", 0.1);

        odom_tracker_.reset(new OdometryTracker(frame_id, child_frame_id, cov_pose, cov_twist));
        odom_ = odom_tracker_->getOdometry();

        topic_pub_odometry_ = controller_nh.advertise<nav_msgs::Odometry>("odometry", 1);

        bool broadcast_tf = true;
        controller_nh.getParam("broadcast_tf", broadcast_tf);

        if(broadcast_tf){
            odom_tf_.header.frame_id = frame_id;
            odom_tf_.child_frame_id = child_frame_id;
            tf_broadcast_odometry_.reset(new tf::TransformBroadcaster);
        }

        publish_timer_ = controller_nh.createTimer(ros::Duration(1/publish_rate), &OdometryController::publish, this);
        service_reset_ = controller_nh.advertiseService("reset_odometry", &OdometryController::srv_reset, this);

        return true;
    }
    virtual void starting(const ros::Time& time){
        if(time != stop_time_) odom_tracker_->init(time); // do not init odometry on restart
        reset_ = false;
    }
    virtual void stopping(const ros::Time& time) { stop_time_ = time; }
    virtual void update(const ros::Time& time, const ros::Duration& period){

        updateState();
        geom_->calcDirect(platform_state_);

        odom_tracker_->track(time, period.toSec(), platform_state_.getVelX(), platform_state_.getVelY(), platform_state_.dRotRobRadS);

        boost::mutex::scoped_try_lock lock(mutex_);
        if(lock){
            if(reset_){
                odom_tracker_->init(time);
                reset_ = false;
            }
            odom_ =  odom_tracker_->getOdometry();
        }
    }

    virtual bool srv_reset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        if(!isRunning()){
            res.message = "not running";
            res.success = false;
        }else{
            boost::mutex::scoped_lock lock(mutex_);
            reset_ = true;
            lock.unlock();
            res.success = true;
            ROS_INFO("Resetting odometry to zero.");
        }

        return true;
    }

private:
    PlatformState platform_state_;

    ros::Publisher topic_pub_odometry_;                 // calculated (measured) velocity, rotation and pose (odometry-based) for the robot
    ros::ServiceServer service_reset_;                  // service to reset odometry to zero

    boost::scoped_ptr<tf::TransformBroadcaster> tf_broadcast_odometry_;    // according transformation for the tf broadcaster
    boost::scoped_ptr<OdometryTracker> odom_tracker_;
    ros::Timer publish_timer_;
    nav_msgs::Odometry odom_;
    bool reset_;
    boost::mutex mutex_;
    geometry_msgs::TransformStamped odom_tf_;
    ros::Time stop_time_;

    void publish(const ros::TimerEvent&){
        if(!isRunning()) return;

        boost::mutex::scoped_lock lock(mutex_);

        topic_pub_odometry_.publish(odom_);

        if(tf_broadcast_odometry_){
            // compose and publish transform for tf package
            // compose header
            odom_tf_.header.stamp = odom_.header.stamp;
            // compose data container
            odom_tf_.transform.translation.x = odom_.pose.pose.position.x;
            odom_tf_.transform.translation.y = odom_.pose.pose.position.y;
            odom_tf_.transform.rotation = odom_.pose.pose.orientation;

            // publish the transform (for debugging, conflicts with robot-pose-ekf)
            tf_broadcast_odometry_->sendTransform(odom_tf_);
        }
    }
};

}

//PLUGINLIB_EXPORT_CLASS( cob_tricycle_controller::WheelController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS( cob_tricycle_controller::OdometryController, controller_interface::ControllerBase)
