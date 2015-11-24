/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_frame_tracker
 *
 * \author
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This class provides a twist_generator for tracking a given tf-frame
 *
 ****************************************************************/
#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <cob_frame_tracker/cob_frame_tracker.h>
#include <cob_frame_tracker/FrameTrackingAction.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>


bool CobFrameTracker::initialize()
{
    ros::NodeHandle nh_;
    ros::NodeHandle nh_tracker("frame_tracker");
    ros::NodeHandle nh_twist("twist_controller");

    ///get params
    if (nh_tracker.hasParam("update_rate"))
    {    nh_tracker.getParam("update_rate", update_rate_);    }
    else
    {    update_rate_ = 68.0;    }    //hz

    if (nh_.hasParam("chain_base_link"))
    {
        nh_.getParam("chain_base_link", chain_base_);
    }
    else
    {
        ROS_ERROR("No chain_base_link specified. Aborting!");
        return false;
    }

    if (nh_.hasParam("chain_tip_link"))
    {
        nh_.getParam("chain_tip_link", chain_tip_link_);
    }
    else
    {
        ROS_ERROR("No chain_tip_link specified. Aborting!");
        return false;
    }

    if(!nh_.getParam("joint_names", joints_))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromParam("/robot_description", tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    tree.getChain(chain_base_, chain_tip_link_, chain_);
    if(chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    //initialize variables and current joint values and velocities
    dof_ = chain_.getNrOfJoints();
    last_q_ = KDL::JntArray(dof_);
    last_q_dot_ = KDL::JntArray(dof_);

    if (nh_tracker.hasParam("movable_trans"))
    {    nh_tracker.getParam("movable_trans", movable_trans_);    }
    else
    {    movable_trans_ = true;    }
    if (nh_tracker.hasParam("movable_rot"))
    {    nh_tracker.getParam("movable_rot", movable_rot_);    }
    else
    {    movable_rot_ = true;    }

    if (nh_tracker.hasParam("max_vel_lin"))
    {    nh_tracker.getParam("max_vel_lin", max_vel_lin_);    }
    else
    {    max_vel_lin_ = 10.0;    }    //m/sec

    if (nh_tracker.hasParam("max_vel_rot"))
    {    nh_tracker.getParam("max_vel_rot", max_vel_rot_);    }
    else
    {    max_vel_rot_ = 6.28;    }    //rad/sec

    // Load PID Controller using gains set on parameter server
    pid_controller_trans_x_.init(ros::NodeHandle(nh_tracker, "pid_trans_x"));
    pid_controller_trans_x_.reset();
    pid_controller_trans_y_.init(ros::NodeHandle(nh_tracker, "pid_trans_y"));
    pid_controller_trans_y_.reset();
    pid_controller_trans_z_.init(ros::NodeHandle(nh_tracker, "pid_trans_z"));
    pid_controller_trans_z_.reset();

    pid_controller_rot_x_.init(ros::NodeHandle(nh_tracker, "pid_rot_x"));
    pid_controller_rot_x_.reset();
    pid_controller_rot_y_.init(ros::NodeHandle(nh_tracker, "pid_rot_y"));
    pid_controller_rot_y_.reset();
    pid_controller_rot_z_.init(ros::NodeHandle(nh_tracker, "pid_rot_z"));
    pid_controller_rot_z_.reset();

    tracking_ = false;
    tracking_goal_ = false;
    tracking_frame_ = chain_tip_link_;

    //ABORTION CRITERIA:
    cart_min_dist_threshold_lin_ = 0.01;
    cart_min_dist_threshold_rot_ = 0.01;
    twist_dead_threshold_lin_ = 0.05;
    twist_dead_threshold_rot_ = 0.05;
    twist_deviation_threshold_lin_ = 0.5;
    twist_deviation_threshold_rot_ = 0.5;

    cart_distance_ = 0.0;
    rot_distance_ = 0.0;

    current_twist_.Zero();
    target_twist_.Zero();

    abortion_counter_ = 0;
    max_abortions_ = update_rate_;    //if tracking fails for 1 minute

    reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_frame_tracker::FrameTrackerConfig>(reconfig_mutex_, nh_tracker));
    reconfigure_server_->setCallback(boost::bind(&CobFrameTracker::reconfigureCallback,   this, _1, _2));

    jointstate_sub_ = nh_.subscribe("joint_states", 1, &CobFrameTracker::jointstateCallback, this);
    twist_pub_ = nh_twist.advertise<geometry_msgs::TwistStamped> ("command_twist_stamped", 1);

    start_server_ = nh_tracker.advertiseService("start_tracking", &CobFrameTracker::startTrackingCallback, this);
    stop_server_ = nh_tracker.advertiseService("stop_tracking", &CobFrameTracker::stopTrackingCallback, this);

    action_name_ = "tracking_action";
    as_.reset(new SAS_FrameTrackingAction_t(nh_tracker, action_name_, false));
    as_->registerGoalCallback(boost::bind(&CobFrameTracker::goalCB, this));
    as_->registerPreemptCallback(boost::bind(&CobFrameTracker::preemptCB, this));
    as_->start();

    timer_ = nh_.createTimer(ros::Duration(1/update_rate_), &CobFrameTracker::run, this);
    timer_.start();

    return true;
}

void CobFrameTracker::run(const ros::TimerEvent& event)
{
    ros::Duration period = event.current_real - event.last_real;

    if(tracking_)
    {
        if (tracking_goal_) // tracking on action goal.
        {
            int status = checkStatus();

            if (status > 0)
            {
                action_success();
            }
            else if (status < 0)
            {
                action_abort();
            }
            else
            {
                //action still active - publish feedback
                if (as_->isActive()){ as_->publishFeedback(action_feedback_); }
            }
        }
        else // tracking on service call
        {
            int status = checkServiceCallStatus();
            if(status < 0)
            {
                this->publishHoldTwist(period);
            }

            ht_.hold = abortion_counter_ >= max_abortions_; // only for service call in case of action ht_.hold = false. What to do with actions?
        }

        publishTwist(period, !ht_.hold); // if not publishing then just update data!
    }
}

bool CobFrameTracker::getTransform(const std::string& target_frame, const std::string& source_frame, tf::StampedTransform& stamped_tf)
{
    bool success = true;
    try
    {
        tf_listener_.lookupTransform(target_frame, source_frame, ros::Time(0), stamped_tf);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("CobFrameTracker::getTransform: \n%s",ex.what());
        success = false;
    }

    return success;
}

void CobFrameTracker::publishZeroTwist()
{
    //publish zero Twist for stopping
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = chain_tip_link_;
    twist_pub_.publish(twist_msg);
}

void CobFrameTracker::publishTwist(ros::Duration period, bool do_publish)
{
    tf::StampedTransform transform_tf;
    geometry_msgs::TwistStamped twist_msg;
    double roll, pitch, yaw;

    if(!this->getTransform(chain_tip_link_, tracking_frame_, transform_tf))
    {
        return;
    }

    if(movable_trans_)
    {
        twist_msg.twist.linear.x = pid_controller_trans_x_.computeCommand(transform_tf.getOrigin().x(), period);
        twist_msg.twist.linear.y = pid_controller_trans_y_.computeCommand(transform_tf.getOrigin().y(), period);
        twist_msg.twist.linear.z = pid_controller_trans_z_.computeCommand(transform_tf.getOrigin().z(), period);
    }

    if(movable_rot_)
    {
        ///ToDo: Consider angular error as RPY or Quaternion?
        ///ToDo: What to do about sign conversion (pi->-pi) in angular rotation?

        twist_msg.twist.angular.x = pid_controller_rot_x_.computeCommand(transform_tf.getRotation().x(), period);
        twist_msg.twist.angular.y = pid_controller_rot_y_.computeCommand(transform_tf.getRotation().y(), period);
        twist_msg.twist.angular.z = pid_controller_rot_z_.computeCommand(transform_tf.getRotation().z(), period);
    }

    twist_msg.header.frame_id = chain_tip_link_;
    twist_msg.header.stamp = ros::Time::now();

    /////debug only
    //if(std::fabs(transform_tf.getOrigin().x()) >= max_vel_lin_)
        //ROS_WARN("Twist.linear.x: %f exceeds limit %f", transform_tf.getOrigin().x(), max_vel_lin_);
    //if(std::fabs(transform_tf.getOrigin().y()) >= max_vel_lin_)
        //ROS_WARN("Twist.linear.y: %f exceeds limit %f", transform_tf.getOrigin().y(), max_vel_lin_);
    //if(std::fabs(transform_tf.getOrigin().z()) >= max_vel_lin_)
        //ROS_WARN("Twist.linear.z: %f exceeds limit %f", transform_tf.getOrigin().z(), max_vel_lin_);
    //if(std::fabs(transform_tf.getOrigin().x()) >= max_vel_rot_)
        //ROS_WARN("Twist.angular.x: %f exceeds limit %f", transform_tf.getOrigin().x(), max_vel_rot_);
    //if(std::fabs(transform_tf.getOrigin().y()) >= max_vel_rot_)
        //ROS_WARN("Twist.angular.y: %f exceeds limit %f", transform_tf.getOrigin().y(), max_vel_rot_);
    //if(std::fabs(transform_tf.getOrigin().z()) >= max_vel_rot_)
        //ROS_WARN("Twist.angular.z: %f exceeds limit %f", transform_tf.getOrigin().z(), max_vel_rot_);

    //twist_msg.twist.linear.x = copysign(std::min(max_vel_lin_, std::fabs(transform_tf.getOrigin().x())),transform_tf.getOrigin().x());
    //twist_msg.twist.linear.y = copysign(std::min(max_vel_lin_, std::fabs(transform_tf.getOrigin().y())),transform_tf.getOrigin().y());
    //twist_msg.twist.linear.z = copysign(std::min(max_vel_lin_, std::fabs(transform_tf.getOrigin().z())),transform_tf.getOrigin().z());
    //twist_msg.twist.angular.x = copysign(std::min(max_vel_rot_, std::fabs(transform_tf.getRotation().x())),transform_tf.getRotation().x());
    //twist_msg.twist.angular.y = copysign(std::min(max_vel_rot_, std::fabs(transform_tf.getRotation().y())),transform_tf.getRotation().y());
    //twist_msg.twist.angular.z = copysign(std::min(max_vel_rot_, std::fabs(transform_tf.getRotation().z())),transform_tf.getRotation().z());

    //eukl distance
    cart_distance_ = sqrt(pow(transform_tf.getOrigin().x(),2) + pow(transform_tf.getOrigin().y(),2) + pow(transform_tf.getOrigin().z(),2));

    ////rot distance
    ////TODO: change to cartesian rot
    //rot_distance_ = 2* acos(transform_msg.transform.rotation.w);

    //get target_twist
    target_twist_.vel.x(twist_msg.twist.linear.x);
    target_twist_.vel.y(twist_msg.twist.linear.y);
    target_twist_.vel.z(twist_msg.twist.linear.z);
    target_twist_.rot.x(twist_msg.twist.angular.x);
    target_twist_.rot.y(twist_msg.twist.angular.y);
    target_twist_.rot.z(twist_msg.twist.angular.z);

    if(do_publish)
        twist_pub_.publish(twist_msg);
}

void CobFrameTracker::publishHoldTwist(const ros::Duration& period)
{
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = chain_tip_link_;

    if(!this->ht_.hold)
    {
        ROS_ERROR_STREAM_THROTTLE(1, "Abortion active: Publishing zero twist");
        ht_.hold = this->getTransform(chain_base_, chain_tip_link_, ht_.transform_tf);
    }
    else
    {
        ROS_ERROR_STREAM_THROTTLE(1, "Abortion active: Publishing hold posture twist");
        tf::StampedTransform new_tf;
        if(this->getTransform(chain_base_, chain_tip_link_, new_tf))
        {
            twist_msg.header.frame_id = chain_tip_link_;
            twist_msg.twist.linear.x = pid_controller_trans_x_.computeCommand(ht_.transform_tf.getOrigin().x() - new_tf.getOrigin().x(), period);
            twist_msg.twist.linear.y = pid_controller_trans_y_.computeCommand(ht_.transform_tf.getOrigin().y() - new_tf.getOrigin().y(), period);
            twist_msg.twist.linear.z = pid_controller_trans_z_.computeCommand(ht_.transform_tf.getOrigin().z() - new_tf.getOrigin().z(), period);

            twist_msg.twist.angular.x = pid_controller_rot_x_.computeCommand(ht_.transform_tf.getRotation().x() - new_tf.getRotation().x(), period);
            twist_msg.twist.angular.y = pid_controller_rot_y_.computeCommand(ht_.transform_tf.getRotation().y() - new_tf.getRotation().y(), period);
            twist_msg.twist.angular.z = pid_controller_rot_z_.computeCommand(ht_.transform_tf.getRotation().z() - new_tf.getRotation().z(), period);
        }
    }

    twist_pub_.publish(twist_msg);
}

bool CobFrameTracker::startTrackingCallback(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response)
{
    if (tracking_)
    {
        ROS_ERROR("CobFrameTracker start was denied! FrameTracker is already tracking a goal");
        response.success = false;
        response.message = "FrameTracker is already tracking goal!";
        return false;
    }
    else
    {
        ROS_INFO("CobFrameTracker started with CART_DIST_SECURITY MONITORING enabled");
        tracking_ = true;
        tracking_goal_ = false;
        tracking_frame_ = request.data;

        response.success = true;
        return true;
    }
}

bool CobFrameTracker::stopTrackingCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (tracking_)
    {
        if (tracking_goal_)
        {
            ROS_ERROR("CobFrameTracker stop was denied because TrackingAction is tracking a goal. You must send 'cancel goal' to the action server instead.");
            return false;
        }
        ROS_INFO("CobFrameTracker stopped successfully");
        tracking_ = false;
        tracking_frame_ = chain_tip_link_;

        publishZeroTwist();
        return true;
    }
    else
    {
        ROS_WARN("CobFrameTracker stop denied because nothing was tracked.");
        return false;
    }
}

void CobFrameTracker::goalCB()
{
    ROS_INFO("Received a new goal");
    if (as_->isNewGoalAvailable())
    {
        boost::shared_ptr<const cob_frame_tracker::FrameTrackingGoal> goal_= as_->acceptNewGoal();
        tracking_frame_ = goal_->tracking_frame;
        tracking_duration_ = goal_->tracking_duration;
        stop_on_goal_ = goal_->stop_on_goal;
        tracking_ = true;
        tracking_goal_ = true;
        abortion_counter_ = 0;
        tracking_start_time_ = ros::Time::now();
    }
}

void CobFrameTracker::preemptCB()
{
    ROS_WARN("Received a preemption request");
    action_result_.success = true;
    action_result_.message = "Action has been preempted";
    as_->setPreempted(action_result_);
    tracking_ = false;
    tracking_goal_ = false;
    tracking_frame_ = chain_tip_link_;

    publishZeroTwist();
}

void CobFrameTracker::action_success()
{
    ROS_INFO("Goal succeeded!");
    as_->setSucceeded(action_result_, action_result_.message);

    tracking_ = false;
    tracking_goal_ = false;
    tracking_frame_ = chain_tip_link_;

    publishZeroTwist();
}

void CobFrameTracker::action_abort()
{
    ROS_WARN("Goal aborted");
    as_->setAborted(action_result_, action_result_.message);

    tracking_ = false;
    tracking_goal_ = false;
    tracking_frame_ = chain_tip_link_;

    publishZeroTwist();
}

int CobFrameTracker::checkStatus()
{
    int status = 0;

    if(ros::Time::now() > tracking_start_time_ + ros::Duration(tracking_duration_))
    {
        action_result_.success = true;
        action_result_.message = std::string("Successfully tracked goal for %f seconds", tracking_duration_);
        status = 1;
    }

    bool infinitesimal_twist = checkInfinitesimalTwist(current_twist_);
    bool distance_violation = checkCartDistanceViolation(cart_distance_, 0.0);
    bool twist_violation = checkTwistViolation(current_twist_, target_twist_);

    if(stop_on_goal_)
    {
        ///ToDo: better metric for when goal is reached
        if(infinitesimal_twist && !distance_violation && !twist_violation)
        {
            action_result_.success = true;
            action_result_.message = "Successfully reached goal";
            status = 2;
        }
    }

    if(distance_violation || twist_violation)
    {
        ROS_ERROR_STREAM("distance_violation || twist_violation");
        abortion_counter_++;
    }

    if(abortion_counter_ > max_abortions_)
    {
        action_result_.success = false;
        action_result_.message = "Constraints violated. Action aborted";
        status = -1;
    }

    return status;
}


int CobFrameTracker::checkServiceCallStatus()
{
    int status = 0;

    bool distance_violation = checkCartDistanceViolation(cart_distance_, 0.0);

    if(distance_violation)
    {
        abortion_counter_++;
    }
    else
    {
        abortion_counter_ = abortion_counter_ > 0 ? abortion_counter_ - 1 : 0;
    }

    if(abortion_counter_ >= max_abortions_)
    {
        abortion_counter_ = max_abortions_;
        status = -1;
    }

    return status;
}


void CobFrameTracker::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = last_q_;
    KDL::JntArray q_dot_temp = last_q_dot_;
    int count = 0;
    for(unsigned int j = 0; j < dof_; j++)
    {
        for(unsigned int i = 0; i < msg->name.size(); i++)
        {
            if(strcmp(msg->name[i].c_str(), joints_[j].c_str()) == 0)
            {
                q_temp(j) = msg->position[i];
                q_dot_temp(j) = msg->velocity[i];
                count++;
                break;
            }
        }
    }
    if(count == joints_.size())
    {
        last_q_ = q_temp;
        last_q_dot_ = q_dot_temp;
        ///---------------------------------------------------------------------
        KDL::FrameVel FrameVel;
        KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(last_q_, last_q_dot_);
        jntToCartSolver_vel_.reset(new KDL::ChainFkSolverVel_recursive(chain_));
        int ret = jntToCartSolver_vel_->JntToCart(jntArrayVel,FrameVel,-1);
        if(ret>=0)
        {
            KDL::Twist twist = FrameVel.GetTwist();
            current_twist_ = twist;
        }
        else
        {
            ROS_ERROR("ChainFkSolverVel failed!");
        }
        ///---------------------------------------------------------------------
    }
}

void CobFrameTracker::reconfigureCallback(cob_frame_tracker::FrameTrackerConfig& config, uint32_t level)
{
    cart_min_dist_threshold_lin_ = config.cart_min_dist_threshold_lin;
    cart_min_dist_threshold_rot_ = config.cart_min_dist_threshold_rot;
    twist_dead_threshold_lin_ = config.twist_dead_threshold_lin;
    twist_dead_threshold_rot_ = config.twist_dead_threshold_rot;
    twist_deviation_threshold_lin_ = config.twist_deviation_threshold_lin;
    twist_deviation_threshold_rot_ = config.twist_deviation_threshold_rot;
}

/** checks whether the twist is infinitesimally small **/
bool CobFrameTracker::checkInfinitesimalTwist(const KDL::Twist current)
{
    if (fabs(current.vel.x()) > twist_dead_threshold_lin_)
    {
        return false;
    }
    if (fabs(current.vel.y()) > twist_dead_threshold_lin_)
    {
        return false;
    }
    if (fabs(current.vel.z()) > twist_dead_threshold_lin_)
    {
        return false;
    }
    if (fabs(current.rot.x()) > twist_dead_threshold_rot_)
    {
        return false;
    }
    if (fabs(current.rot.x()) > twist_dead_threshold_rot_)
    {
        return false;
    }
    if (fabs(current.rot.x()) > twist_dead_threshold_rot_)
    {
        return false;
    }

    ///all twist velocities are <= dead_threshold -> twist is infinitesimal
    return true;
}

/** checks whether the Cartesian distance between tip and target frame is ok **/
bool CobFrameTracker::checkCartDistanceViolation(const double dist, const double rot)
{
    if (dist > cart_min_dist_threshold_lin_)
    {
        return true;
    }
    if (rot > cart_min_dist_threshold_rot_)
    {
        return true;
    }

    ///Cartesian distance is acceptable -> no violation
    return false;
}

/** checks whether the current twist is following the target twist "close enough" **/
bool CobFrameTracker::checkTwistViolation(const KDL::Twist current, const KDL::Twist target)
{
    if (fabs(current.vel.x() - target.vel.x()) > twist_deviation_threshold_lin_)
    {
        return true;
    }
    if (fabs(current.vel.y() - target.vel.y()) > twist_deviation_threshold_lin_)
    {
        return true;
    }
    if (fabs(current.vel.z() - target.vel.z()) > twist_deviation_threshold_lin_)
    {
        return true;
    }
    if (fabs(current.rot.x() - target.rot.x()) > twist_deviation_threshold_rot_)
    {
        return true;
    }
    if (fabs(current.rot.y() - target.rot.y()) > twist_deviation_threshold_rot_)
    {
        return true;
    }
    if (fabs(current.rot.z() - target.rot.z()) > twist_deviation_threshold_rot_)
    {
        return true;
    }

    ///Cartesian Twist distance is acceptable -> no violation
    return false;
}
