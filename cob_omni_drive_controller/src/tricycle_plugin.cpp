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


#include "GeomController.h"
#include "WheelControllerBase.h"

#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>
#include <cob_omni_drive_controller/OdometryTracker.h>

#include <dynamic_reconfigure/server.h>
#include <cob_omni_drive_controller/SteerCtrlConfig.h>

#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>

#include <pluginlib/class_list_macros.h>

namespace cob_omni_drive_controller
{

template<typename Interface, typename Controller> class TricycleGeomController:
    public GeomControllerBase<typename Interface::ResourceHandleType, Controller>,
        public controller_interface::Controller<Interface> {
public:
    typedef std::vector<typename Controller::WheelParams> wheel_params_t;
    bool init(Interface* hw, ros::NodeHandle& controller_nh){
        std::vector<typename Controller::WheelParams> wheel_params;
        if(!parseWheelParams(wheel_params, controller_nh)) return false;
        return init(hw, wheel_params);
    }
    bool init(Interface* hw, const wheel_params_t & wheel_params){
        if(!this->setup(wheel_params)) return false;
        try{
            //for (unsigned i=0; i<wheel_params.size(); i++){
                //this->steer_joints_.push_back(hw->getHandle(wheel_params[i].geom.steer_name));
                //this->drive_joints_.push_back(hw->getHandle(wheel_params[i].geom.drive_name));
            //}
            this->steer_joints_.push_back(typename Interface::ResourceHandleType(hardware_interface::JointStateHandle("fl_caster_rotation_joint", &fake_steer_pos_[0], &fake_steer_vel_[0], &fake_steer_eff_[0]), &fake_steer_vel_[0]));
            this->drive_joints_.push_back(typename Interface::ResourceHandleType(hardware_interface::JointStateHandle("fl_caster_r_wheel_joint", &fake_drive_pos_[0], &fake_drive_vel_[0], &fake_drive_eff_[0]), &fake_drive_vel_[0]));
            this->steer_joints_.push_back(hw->getHandle("b_caster_rotation_joint"));
            this->drive_joints_.push_back(hw->getHandle("b_caster_r_wheel_joint"));
            this->steer_joints_.push_back(typename Interface::ResourceHandleType(hardware_interface::JointStateHandle("fr_caster_rotation_joint", &fake_steer_pos_[1], &fake_steer_vel_[1], &fake_steer_eff_[1]), &fake_steer_vel_[1]));
            this->drive_joints_.push_back(typename Interface::ResourceHandleType(hardware_interface::JointStateHandle("fr_caster_r_wheel_joint", &fake_drive_pos_[1], &fake_drive_vel_[1], &fake_drive_eff_[1]), &fake_drive_vel_[1]));
        }
        catch(const std::exception &e){
            ROS_ERROR_STREAM("Error while attaching handles: " << e.what());
            return false;
        }
        return true;
    }
protected:
    double fake_steer_pos_[2], fake_drive_pos_[2], fake_steer_vel_[2], fake_drive_vel_[2], fake_steer_eff_[2], fake_drive_eff_[2]; // set zero in contructor
};


class TricycleController : public WheelControllerBase< TricycleGeomController<hardware_interface::VelocityJointInterface, UndercarriageCtrl> >
{
public:
    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){
        //common part
        wheel_params_t wheel_params;
        if(!parseWheelParams(wheel_params, controller_nh) || !TricycleGeomController::init(hw, wheel_params)) return false;

        //odometry part
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

        publish_timer_ = controller_nh.createTimer(ros::Duration(1/publish_rate), &TricycleController::publish, this);
        service_reset_ = controller_nh.advertiseService("reset_odometry", &TricycleController::srv_reset, this);
        
        //control part
        pos_ctrl_.init(wheel_params, controller_nh);

        return setup(root_nh, controller_nh);
    }
    virtual void update(const ros::Time& time, const ros::Duration& period){
        //common part
        updateState();

        //odometry part
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

        //control part
        pos_ctrl_.try_configure(*geom_);

        updateCtrl(time, period);

        ////for (unsigned i=0; i<wheel_commands_.size(); i++){
            ////steer_joints_[i].setCommand(wheel_commands_[i].dVelGearSteerRadS);
            ////drive_joints_[i].setCommand(wheel_commands_[i].dVelGearDriveRadS);
        ////}
        steer_joints_[0].setCommand(0.0);
        steer_joints_[1].setCommand(wheel_commands_[1].dVelGearSteerRadS);
        steer_joints_[2].setCommand(0.0);
        drive_joints_[0].setCommand(cos(steer_joints_[1].getPosition()) * drive_joints_[1].getVelocity());
        drive_joints_[1].setCommand(wheel_commands_[1].dVelGearDriveRadS);
        drive_joints_[2].setCommand(cos(steer_joints_[1].getPosition()) * drive_joints_[1].getVelocity());
    }
    virtual void starting(const ros::Time& time){
        if(time != stop_time_) odom_tracker_->init(time); // do not init odometry on restart
        reset_ = false;
    }
    virtual void stopping(const ros::Time& time) { stop_time_ = time; }

//control part
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
                copy(config, params.front().pos_ctrl);
                reconfigure_server_->setConfigDefault(config);
            }

            for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                boost::shared_ptr<dynamic_reconfigure::Server<SteerCtrlConfig> > dr(new dynamic_reconfigure::Server<SteerCtrlConfig>(mutex, ros::NodeHandle(nh, params[i].geom.steer_name)));
                SteerCtrlConfig config;
                copy(config, params[i].pos_ctrl);
                dr->setConfigDefault(config);
                dr->updateConfig(config);
                dr->setCallback(boost::bind(&PosCtrl::setForOne, this, i, _1, _2));  // this writes the joint-specific config into pos_ctrl_params
                reconfigure_server_axes_.push_back(dr);
            }
        }
    private:
        static void copy(PosCtrlParams &params, const SteerCtrlConfig &config){
            params.dSpring = config.spring;
            params.dDamp = config.damp;
            params.dVirtM = config.virt_mass;
            params.dDPhiMax = config.d_phi_max;
            params.dDDPhiMax = config.dd_phi_max;
        }
        static void copy(SteerCtrlConfig &config, const PosCtrlParams &params){
            config.spring = params.dSpring;
            config.damp = params.dDamp;
            config.virt_mass = params.dVirtM;
            config.d_phi_max = params.dDPhiMax;
            config.dd_phi_max = params.dDDPhiMax;
        }
        // these function don't get locked
        void setForAll(SteerCtrlConfig &config, uint32_t /*level*/) {
            ROS_INFO("configure all steers: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
            for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                copy(pos_ctrl_params[i], config);
                if(!reconfigure_server_axes_.empty()){
                    reconfigure_server_axes_[i]->setConfigDefault(config);
                    reconfigure_server_axes_[i]->updateConfig(config);
                }
            }
            updated = true;
        }
        void setForOne(size_t i, SteerCtrlConfig &config, uint32_t /*level*/) {
            ROS_INFO("configure steer %d: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", (int)i, config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
            copy(pos_ctrl_params[i], config);
            updated = true;
        }
        std::vector<PosCtrlParams> pos_ctrl_params;
        boost::recursive_mutex mutex; // dynamic_reconfigure::Server calls the callback from the setCallback function
        bool updated;
        boost::scoped_ptr< dynamic_reconfigure::Server<SteerCtrlConfig> > reconfigure_server_;
        std::vector<boost::shared_ptr< dynamic_reconfigure::Server<SteerCtrlConfig> > > reconfigure_server_axes_; // TODO: use unique_ptr
    } pos_ctrl_;

//odometry part
public:
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

PLUGINLIB_EXPORT_CLASS( cob_omni_drive_controller::TricycleController, controller_interface::ControllerBase)
