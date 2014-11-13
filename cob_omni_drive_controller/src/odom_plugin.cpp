
#include <hardware_interface/joint_state_interface.h>

#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>

#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>
#include <cob_omni_drive_controller/OdometryTracker.h>

#include "GeomController.h"

namespace cob_omni_drive_controller
{

// this controller gets access to the JointStateInterface
class OdometryController: public GeomController<hardware_interface::JointStateInterface>
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
        odom_tracker_.reset(new OdometryTracker);

        bool broadcast_tf = true;
        controller_nh.getParam("broadcast_tf", broadcast_tf);

        if(broadcast_tf){
            odom_tf_.header.frame_id = "/odom_combined";
            odom_tf_.child_frame_id = "/base_footprint";
            tf_broadcast_odometry_.reset(new tf::TransformBroadcaster);
        }

        publish_timer_ = controller_nh.createTimer(ros::Duration(publish_rate), &OdometryController::publish, this);

        return true;
  }
    virtual void starting(const ros::Time& time){
        geom_->reset();
        odom_tracker_->init(time);
    }
    virtual void update(const ros::Time& time, const ros::Duration& period){

        GeomController::update();

        geom_->calcDirect(platform_state_);

        odom_tracker_->track(time, period.toSec(), platform_state_.get_vel_x(), platform_state_.get_vel_y(), platform_state_.dRotRobRadS);
    }
    virtual void stopping(const ros::Time& time) {}

private:
    UndercarriageCtrlGeom::PlatformState platform_state_;

    ros::Publisher topic_pub_odometry_;                 // calculated (measured) velocity, rotation and pose (odometry-based) for the robot

    boost::scoped_ptr<tf::TransformBroadcaster> tf_broadcast_odometry_;    // according transformation for the tf broadcaster
    boost::scoped_ptr<OdometryTracker> odom_tracker_;
    ros::Timer publish_timer_;
    geometry_msgs::TransformStamped odom_tf_;
  
  
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
