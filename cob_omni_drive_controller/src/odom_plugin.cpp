
#include <hardware_interface/joint_state_interface.h>

#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>

#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>
#include <cob_omni_drive_controller/OdometryTracker.h>

#include <cob_srvs/Trigger.h>

#include "GeomController.h"

namespace cob_omni_drive_controller
{

// this controller gets access to the JointStateInterface
class OdometryController: public GeomController<hardware_interface::JointStateInterface, UndercarriageGeom>
{
public:
    OdometryController() {}

    virtual bool init(hardware_interface::JointStateInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){

        if(!GeomController::init(hw, controller_nh)) return false;

        double publish_rate;
        if (!controller_nh.getParam("publish_rate", publish_rate)){
            ROS_ERROR("Parameter 'publish_rate' not set");
            return false;
        }
        if(publish_rate <= 0){
            ROS_ERROR_STREAM("publish_rate must be positive.");
            return false;
        }

        odom_tracker_.reset(new OdometryTracker);

        topic_pub_odometry_ = controller_nh.advertise<nav_msgs::Odometry>("odometry", 1);

        bool broadcast_tf = true;
        controller_nh.getParam("broadcast_tf", broadcast_tf);

        if(broadcast_tf){
            odom_tf_.header.frame_id = "/odom_combined";
            odom_tf_.child_frame_id = "/base_footprint";
            tf_broadcast_odometry_.reset(new tf::TransformBroadcaster);
        }

        publish_timer_ = controller_nh.createTimer(ros::Duration(1/publish_rate), &OdometryController::publish, this);
	service_reset_ = controller_nh.advertiseService("reset_odometry", &OdometryController::srv_reset, this);

        return true;
    }
    virtual void starting(const ros::Time& time){
        GeomController::reset();
        if(time != stop_time_) odom_tracker_->init(time); // do not init odometry on restart
    }

    virtual bool srv_reset(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
    {
        if(!isRunning()){
            res.error_message.data = "not running";
            res.success.data = false;
        }else{
            odom_tracker_->reset(ros::Time::now());
            res.success.data = true;
            ROS_INFO("Resetting odometry to zero.");
        }

        return true;
    }

    virtual void update(const ros::Time& time, const ros::Duration& duration){
        double period = duration.toSec();

        GeomController::update(period);

        geom_->calcDirect(platform_state_);

        odom_tracker_->track(time, period, platform_state_.getVelX(), platform_state_.getVelY(), platform_state_.dRotRobRadS);
    }
    virtual void stopping(const ros::Time& time) { stop_time_ = time; }

private:
    UndercarriageGeom::PlatformState platform_state_;

    ros::Publisher topic_pub_odometry_;                 // calculated (measured) velocity, rotation and pose (odometry-based) for the robot
    ros::ServiceServer service_reset_;			// service to reset odometry to zero

    boost::scoped_ptr<tf::TransformBroadcaster> tf_broadcast_odometry_;    // according transformation for the tf broadcaster
    boost::scoped_ptr<OdometryTracker> odom_tracker_;
    ros::Timer publish_timer_;
    geometry_msgs::TransformStamped odom_tf_;
    ros::Time stop_time_;
  
  
    void publish(const ros::TimerEvent&){
        if(!isRunning()) return;

        nav_msgs::Odometry odom = odom_tracker_->getOdometry();
        topic_pub_odometry_.publish(odom);

        if(tf_broadcast_odometry_){
            // compose and publish transform for tf package
            // compose header
            odom_tf_.header.stamp = odom.header.stamp;
            // compose data container
            odom_tf_.transform.translation.x = odom.pose.pose.position.x;
            odom_tf_.transform.translation.y = odom.pose.pose.position.y;
            odom_tf_.transform.rotation = odom.pose.pose.orientation;

            // publish the transform (for debugging, conflicts with robot-pose-ekf)
            tf_broadcast_odometry_->sendTransform(odom_tf_);
        }
    }
};

}

PLUGINLIB_EXPORT_CLASS( cob_omni_drive_controller::OdometryController, controller_interface::ControllerBase)
