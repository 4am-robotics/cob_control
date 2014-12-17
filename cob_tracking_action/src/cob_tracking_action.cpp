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
 *   ROS package name: cob_tracking_action
 *
 * \author
 *   Author: Christian Ehrmann, email: christian.ehrmann@ipa.fraunhofer.de
 *
 * \date Date of creation: December, 2014
 *
 * \brief
 *   ...
 *
 ****************************************************************/
#include <ros/ros.h>

#include <cob_tracking_action/cob_tracking_action.h>
#include <cob_tracking_action/TrackingAction.h>
#include <math.h>

bool TrackingAction::initialize()
{
	ros::NodeHandle nh_action("tracking_action");

	//FrameTrackingInsert:
	ros::NodeHandle nh_twist("twist_controller");
	ros::NodeHandle nh_cartesian("cartesian_controller");

	///get params
	if (nh_cartesian.hasParam("update_rate"))
	{	nh_cartesian.getParam("update_rate", update_rate_);	}
	else
	{	update_rate_ = 68.0;	}	//hz

	if (nh_cartesian.hasParam("max_vel_lin"))
	{	nh_cartesian.getParam("max_vel_lin", max_vel_lin_);	}
	else
	{	max_vel_lin_ = 10.0;	}	//m/sec

	if (nh_cartesian.hasParam("max_vel_rot"))
	{	nh_cartesian.getParam("max_vel_rot", max_vel_rot_);	}
	else
	{	max_vel_rot_ = 6.28;	}	//rad/sec

	if (nh_cartesian.hasParam("active_frame"))
	{
		nh_cartesian.getParam("active_frame", active_frame_);
	}
	else
	{
		ROS_ERROR("No active_frame specified. Aborting!");
		return false;
	}

	if (nh_cartesian.hasParam("movable_trans"))
	{	nh_cartesian.getParam("movable_trans", movable_trans_);	}
	else
	{	movable_trans_ = true;	}
	if (nh_cartesian.hasParam("movable_rot"))
	{	nh_cartesian.getParam("movable_rot", movable_rot_);	}
	else
	{	movable_rot_ = true;	}

	// Load PID Controller using gains set on parameter server
	pid_controller_trans_x_.init(ros::NodeHandle(nh_cartesian, "pid_trans_x"));
	pid_controller_trans_x_.reset();

	pid_controller_trans_y_.init(ros::NodeHandle(nh_cartesian, "pid_trans_y"));
	pid_controller_trans_y_.reset();

	pid_controller_trans_z_.init(ros::NodeHandle(nh_cartesian, "pid_trans_z"));
	pid_controller_trans_z_.reset();

	pid_controller_rot_.init(ros::NodeHandle(nh_cartesian, "pid_rot"));
	pid_controller_rot_.reset();

	twist_pub_ = nh_twist.advertise<geometry_msgs::Twist> ("command_twist", 1);
	target_tracking_frame_ = active_frame_;
	tracking_ = false;
	//End FrameTrackerInsert:
	//initial feedback:
	feedback_.twist.linear.x = 0.0;
	feedback_.twist.linear.y = 2.0;
	feedback_.twist.linear.z = 3.0;
	feedback_.twist.angular.x = 4.0;
	feedback_.twist.angular.y = 5.0;
	feedback_.twist.angular.z = 6.0;
	feedback_.distance = 10;

	//register the goal and feeback callbacks

	// JointNames
	//if(!nh_.getParam("joint_names", joints_))
	//{
	//	ROS_ERROR("Parameter 'joint_names' not set");
	//	return false;
	//}
	//dof_ = joints_.size();
	//


	ROS_INFO("...initialized!");
	return true;
}


void TrackingAction::run()
{
	//FrameTracker insert on Begin:
	ros::Time time = ros::Time::now();
	ros::Time last_update_time = time;
	ros::Duration period = time - last_update_time;

	ros::Rate r(update_rate_);

	while (ros::ok())
	{
		time = ros::Time::now();
		period = time - last_update_time;
		double maxdurationOftracking = start_of_tracking_.toSec() + double(tracking_time_);

//stamp_ - time_filter_threshold
		if(tracking_) {
			ROS_INFO("[TRACKING GOAL ...]");
			// tracking time is over --> successful
			if (tracking_time_ != 0) {
				ROS_INFO("Server is waiting for trackingtime = %f to end.",maxdurationOftracking-time.toSec());
				if (time.toSec() > maxdurationOftracking) {
					tracking_ = false;
					TrackingAction::succeed();
				}
			}
			// target reaches active frame --> successful
			if (reachable_goal_) {
				ROS_INFO("Goal was reached!");
				result_.success = true;
				TrackingAction::succeed();
			}

			// terms of abortion
			/// do things ...




			publish_twist(period);
		}
		else
		{
			ROS_INFO("[Server is waiting for a new goal!]");

		}


		last_update_time = time;

		ros::spinOnce();
		r.sleep();
		//FrameTracker insert END
//		ROS_INFO("tracking_ = %s", tracking_ ? "true" : "false" );
//		ROS_INFO("cob_tracking_action...spinning");

		if (as_.isActive()) {
			ROS_INFO("Sending feedback:");
			as_.publishFeedback(feedback_);
		}
	}
}


void TrackingAction::goalCB()
{
	ROS_INFO("this is a new goal callback");
	ROS_INFO("Waiting for a new goal");
	if (as_.isNewGoalAvailable()) {
		boost::shared_ptr<const cob_tracking_action::TrackingGoal> goal_= as_.acceptNewGoal();

		//target_tracking_frame_ = as_.acceptNewGoal()->tracking_frame;
		target_tracking_frame_ = goal_->tracking_frame;
		reachable_goal_ = goal_->reachable_goal;
		tracking_time_ = goal_->tracking_time;
		tracking_ = true;
		start_of_tracking_ = ros::Time::now();
		//target_tracking_frame_ = goal_.tracking_frame;
		ROS_INFO("Output: Received target_tracking_frame_ = %s", target_tracking_frame_.c_str());
		ROS_INFO("Output: Received reachable_goal_ = %s", reachable_goal_ ? "true" : "false" );
		ROS_INFO("Output: Received tracking_time_ = %u", tracking_time_);
	}


	//target_tracking_frame_ = goal_.tracking_frame;
}

// if canceled
void TrackingAction::preemptCB()
{
	ROS_INFO("this is the preempt callback");
	as_.setPreempted(result_);
	tracking_ = false;
	target_tracking_frame_ = active_frame_;
	geometry_msgs::Twist twist_msg;
	twist_pub_.publish(twist_msg);
}

void TrackingAction::publish_twist(ros::Duration period)
{
	tf::StampedTransform transform_tf;
	geometry_msgs::TransformStamped transform_msg;
	geometry_msgs::Twist twist_msg;
	ROS_INFO("active_frame_ = %s", active_frame_.c_str());
	ROS_INFO("target_tracking_frame_ = %s", target_tracking_frame_.c_str());

	try{
		tf_listener_.lookupTransform(active_frame_, target_tracking_frame_, ros::Time(0), transform_tf);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}

	tf::transformStampedTFToMsg(transform_tf, transform_msg);

	//my calculations begin

	eukl_dist_lin_ = sqrt(transform_msg.transform.translation.x * transform_msg.transform.translation.x + transform_msg.transform.translation.y * transform_msg.transform.translation.y + transform_msg.transform.translation.z * transform_msg.transform.translation.z);
	//my calculations end
	if(movable_trans_)
	{
		/// Use pid_trans_x .. y .. z as controller parameters. Has to be changed in arm_controller_sim.yaml !
		twist_msg.linear.x = pid_controller_trans_x_.computeCommand(transform_msg.transform.translation.x, period);
		twist_msg.linear.y = pid_controller_trans_y_.computeCommand(transform_msg.transform.translation.y, period);
		twist_msg.linear.z = pid_controller_trans_z_.computeCommand(transform_msg.transform.translation.z, period);
	}

	if(movable_rot_)
	{
		twist_msg.angular.x = pid_controller_rot_.computeCommand(transform_msg.transform.rotation.x, period);
		twist_msg.angular.y = pid_controller_rot_.computeCommand(transform_msg.transform.rotation.y, period);
		twist_msg.angular.z = pid_controller_rot_.computeCommand(transform_msg.transform.rotation.z, period);
	}

	/////debug only
	//if(std::fabs(transform_msg.transform.translation.x) >= max_vel_lin_)
		//ROS_WARN("Twist.linear.x: %f exceeds limit %f", transform_msg.transform.translation.x, max_vel_lin_);
	//if(std::fabs(transform_msg.transform.translation.y) >= max_vel_lin_)
		//ROS_WARN("Twist.linear.y: %f exceeds limit %f", transform_msg.transform.translation.y, max_vel_lin_);
	//if(std::fabs(transform_msg.transform.translation.z) >= max_vel_lin_)
		//ROS_WARN("Twist.linear.z: %f exceeds limit %f", transform_msg.transform.translation.z, max_vel_lin_);
	//if(std::fabs(transform_msg.transform.rotation.x) >= max_vel_rot_)
		//ROS_WARN("Twist.angular.x: %f exceeds limit %f", transform_msg.transform.rotation.x, max_vel_rot_);
	//if(std::fabs(transform_msg.transform.rotation.y) >= max_vel_rot_)
		//ROS_WARN("Twist.angular.y: %f exceeds limit %f", transform_msg.transform.rotation.y, max_vel_rot_);
	//if(std::fabs(transform_msg.transform.rotation.z) >= max_vel_rot_)
		//ROS_WARN("Twist.angular.z: %f exceeds limit %f", transform_msg.transform.rotation.z, max_vel_rot_);

	//twist_msg.linear.x = copysign(std::min(max_vel_lin_, std::fabs(transform_msg.transform.translation.x)),transform_msg.transform.translation.x);
	//twist_msg.linear.y = copysign(std::min(max_vel_lin_, std::fabs(transform_msg.transform.translation.y)),transform_msg.transform.translation.y);
	//twist_msg.linear.z = copysign(std::min(max_vel_lin_, std::fabs(transform_msg.transform.translation.z)),transform_msg.transform.translation.z);
	//twist_msg.angular.x = copysign(std::min(max_vel_rot_, std::fabs(transform_msg.transform.rotation.x)),transform_msg.transform.rotation.x);
	//twist_msg.angular.y = copysign(std::min(max_vel_rot_, std::fabs(transform_msg.transform.rotation.y)),transform_msg.transform.rotation.y);
	//twist_msg.angular.z = copysign(std::min(max_vel_rot_, std::fabs(transform_msg.transform.rotation.z)),transform_msg.transform.rotation.z);

	twist_pub_.publish(twist_msg);
}

void TrackingAction::succeed()
{
	ROS_INFO("Goal succeeded!");
	as_.setSucceeded(result_, "succeeded text");
	tracking_ = false;
	geometry_msgs::Twist twist_msg;
	twist_pub_.publish(twist_msg);
}

void TrackingAction::abort()
{
	ROS_INFO("this is the abort method");
	//as_.setPreempted(result_);
	as_.setAborted(result_);
	tracking_ = false;
	target_tracking_frame_ = active_frame_;
	geometry_msgs::Twist twist_msg;
	twist_pub_.publish(twist_msg);
}
