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


#include <math.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>

#include <cob_tricycle_controller/param_parser.h>
#include <cob_base_controller_utils/OdometryTracker.h>
#include <cob_tricycle_controller/TricycleCtrlTypes.h>

#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>

#include <pluginlib/class_list_macros.h>

#include <urdf/model.h>
#include <tf2/LinearMath/Transform.h>

namespace cob_tricycle_controller
{

// this controller gets access to the JointStateInterface
class OdometryController: public controller_interface::Controller<hardware_interface::JointStateInterface>
{
public:
    OdometryController() {}

    virtual bool init(hardware_interface::JointStateInterface* hw, ros::NodeHandle &nh)
    {
        if (!nh.getParam("steer_joint", wheel_state_.steer_name)){
            ROS_ERROR("Parameter 'steer_joint' not set");
            return false;
        }
        if (!nh.getParam("drive_joint", wheel_state_.drive_name)){
            ROS_ERROR("Parameter 'drive_joint' not set");
            return false;
        }
        steer_joint_ = hw->getHandle(wheel_state_.steer_name);
        drive_joint_ = hw->getHandle(wheel_state_.drive_name);


        urdf::Model model;
        std::string description_name;
        bool has_model = nh.searchParam("robot_description", description_name) &&  model.initParam(description_name);

        urdf::Vector3 steer_pos;
        boost::shared_ptr<const urdf::Joint> steer_joint;

        if(has_model){
            steer_joint = model.getJoint(wheel_state_.steer_name);
            if(steer_joint){
                tf2::Transform transform;
                //root link for tricycle is "base_pivot_link"
                if(parseWheelTransform(wheel_state_.steer_name, model.getRoot()->name, transform, &model)){
                    wheel_state_.pos_x = transform.getOrigin().getX();
                    wheel_state_.pos_y = transform.getOrigin().getY();
                    wheel_state_.radius = transform.getOrigin().getZ();
                    wheel_state_.sign = cos(transform.getRotation().getAngle());
                }
            }
        }

        double publish_rate;
        if (!nh.getParam("publish_rate", publish_rate)){
            ROS_ERROR("Parameter 'publish_rate' not set");
            return false;
        }
        if(publish_rate <= 0){
            ROS_ERROR_STREAM("publish_rate must be positive.");
            return false;
        }

        const std::string frame_id = nh.param("frame_id", std::string("odom"));
        const std::string child_frame_id = nh.param("child_frame_id", std::string("base_footprint"));
        const double cov_pose = nh.param("cov_pose", 0.1);
        const double cov_twist = nh.param("cov_twist", 0.1);

        odom_tracker_.reset(new OdometryTracker(frame_id, child_frame_id, cov_pose, cov_twist));
        odom_ = odom_tracker_->getOdometry();

        topic_pub_odometry_ = nh.advertise<nav_msgs::Odometry>("odometry", 1);

        bool broadcast_tf = true;
        nh.getParam("broadcast_tf", broadcast_tf);

        if(broadcast_tf){
            odom_tf_.header.frame_id = frame_id;
            odom_tf_.child_frame_id = child_frame_id;
            tf_broadcast_odometry_.reset(new tf::TransformBroadcaster);
        }

        publish_timer_ = nh.createTimer(ros::Duration(1/publish_rate), &OdometryController::publish, this);
        service_reset_ = nh.advertiseService("reset_odometry", &OdometryController::srv_reset, this);

        return true;
    }
    virtual void starting(const ros::Time& time)
    {
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

    virtual void update(const ros::Time& time, const ros::Duration& period)
    {
        updateState();

        odom_tracker_->track(time, period.toSec(), platform_state_.velX, platform_state_.velY, platform_state_.rotTheta);

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
    WheelState wheel_state_;
    hardware_interface::JointStateHandle steer_joint_;
    hardware_interface::JointStateHandle drive_joint_;

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

    void updateState(){
        //get JointState from JointHandles
        wheel_state_.steer_pos = steer_joint_.getPosition();
        wheel_state_.steer_vel = steer_joint_.getVelocity();
        wheel_state_.drive_pos = drive_joint_.getPosition();
        wheel_state_.drive_vel = drive_joint_.getVelocity();

        //calculate forward kinematics
        // http://www.wolframalpha.com/input/?i=Solve%5Bx%3D%3Dw*cos(a),+phi%3D%3Dw*sin(a)%2Fr,a,w%5D
        double r_base = wheel_state_.pos_x * wheel_state_.sign;
        platform_state_.velX = wheel_state_.radius*wheel_state_.drive_vel*cos(wheel_state_.steer_pos);
        platform_state_.velY = 0.0;
        platform_state_.rotTheta = wheel_state_.radius*wheel_state_.drive_vel*sin(wheel_state_.steer_pos)/r_base;
    }
};

}

PLUGINLIB_EXPORT_CLASS( cob_tricycle_controller::OdometryController, controller_interface::ControllerBase)
