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


#include <hardware_interface/joint_state_interface.h>

#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>

#include <cob_base_controller_utils/OdometryTracker.h>
#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>

#include <std_srvs/Trigger.h>

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

        frame_id_ = controller_nh.param("frame_id", std::string("odom"));
        child_frame_id_ = controller_nh.param("child_frame_id", std::string("base_footprint"));
        const double cov_pose = controller_nh.param("cov_pose", 0.1);
        const double cov_twist = controller_nh.param("cov_twist", 0.1);

        odom_tracker_.reset(new OdometryTracker(frame_id_, child_frame_id_, cov_pose, cov_twist));
        odom_ = odom_tracker_->getOdometry();

        topic_pub_odometry_ = controller_nh.advertise<nav_msgs::Odometry>("odometry", 1);

        bool broadcast_tf = true;
        controller_nh.getParam("broadcast_tf", broadcast_tf);

        if(broadcast_tf){
            odom_tf_.header.frame_id = frame_id_;
            odom_tf_.child_frame_id = child_frame_id_;
            tf_broadcast_odometry_.reset(new tf::TransformBroadcaster);
        }

        controller_nh.getParam("invert_odom_tf", invert_odom_tf_);

        publish_timer_ = controller_nh.createTimer(ros::Duration(1 / publish_rate), &OdometryController::publish, this);
        service_reset_ = controller_nh.advertiseService("reset_odometry", &OdometryController::srv_reset, this);

        return true;
    }
    virtual void starting(const ros::Time& time){
        if(time != stop_time_) odom_tracker_->init(time); // do not init odometry on restart
        reset_ = false;
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
    virtual void stopping(const ros::Time& time) { stop_time_ = time; }

private:
    PlatformState platform_state_;

    ros::Publisher topic_pub_odometry_;                 // calculated (measured) velocity, rotation and pose (odometry-based) for the robot
    ros::ServiceServer service_reset_;                  // service to reset odometry to zero

    boost::scoped_ptr<tf::TransformBroadcaster> tf_broadcast_odometry_;    // according transformation for the tf broadcaster
    boost::scoped_ptr<OdometryTracker> odom_tracker_;
    ros::Timer publish_timer_;
    nav_msgs::Odometry odom_;
    bool reset_;
    bool invert_odom_tf_ = false;
    boost::mutex mutex_;
    std::string frame_id_, child_frame_id_;
    geometry_msgs::TransformStamped odom_tf_;
    ros::Time stop_time_;


    void publish(const ros::TimerEvent& event){
        if(!isRunning()) return;

        boost::mutex::scoped_lock lock(mutex_);

        topic_pub_odometry_.publish(odom_);

        if(tf_broadcast_odometry_){
            // check prevents TF_REPEATED_DATA warning
            if (odom_tf_.header.stamp == odom_.header.stamp){
                // ROS_WARN_STREAM("OdometryController: already published odom_tf before");
                // ROS_WARN_STREAM("\todom_tf: " << odom_tf_);
                // ROS_WARN_STREAM("\todom_: " << odom_);
                // ROS_WARN_STREAM("\tevent.last_expected: " << event.last_expected);
                // ROS_WARN_STREAM("\tevent.last_real: " << event.last_real);
                // ROS_WARN_STREAM("\tevent.current_expected: " << event.current_expected);
                // ROS_WARN_STREAM("\tevent.current_real: " << event.current_real);
            } else {
                // compose and publish transform for tf package
                // compose header
                odom_tf_.header.stamp = odom_.header.stamp;
                // compose data container
                odom_tf_.transform.translation.x = odom_.pose.pose.position.x;
                odom_tf_.transform.translation.y = odom_.pose.pose.position.y;
                odom_tf_.transform.rotation = odom_.pose.pose.orientation;
                if (invert_odom_tf_){
                    odom_tf_.header.frame_id = child_frame_id_;
                    odom_tf_.child_frame_id = frame_id_;
                    tf::Transform transform;
                    tf::transformMsgToTF(odom_tf_.transform, transform);
                    tf::transformTFToMsg(transform.inverse(), odom_tf_.transform);
                }
                // publish the transform (for debugging, conflicts with robot-pose-ekf)
                tf_broadcast_odometry_->sendTransform(odom_tf_);
            }
        }
    }
};

}

PLUGINLIB_EXPORT_CLASS( cob_omni_drive_controller::OdometryController, controller_interface::ControllerBase)
