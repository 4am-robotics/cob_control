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
 *   ROS package name: cob_cob_frame_tracker
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

#include <cob_frame_tracker/cob_frame_tracker.h>


bool CobFrameTracker::initialize()
{
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
	
	if (nh_cartesian.hasParam("chain_tip_link"))
	{
		nh_cartesian.getParam("chain_tip_link", chain_tip_link_);
	}
	else
	{
		ROS_ERROR("No chain_tip_link specified. Aborting!");
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
	pid_controller_trans_x_.init(ros::NodeHandle(nh_cartesian, "pid_trans"));
	pid_controller_trans_x_.reset();
	pid_controller_trans_y_.init(ros::NodeHandle(nh_cartesian, "pid_trans"));
	pid_controller_trans_y_.reset();
	pid_controller_trans_z_.init(ros::NodeHandle(nh_cartesian, "pid_trans"));
	pid_controller_trans_z_.reset();
	
	pid_controller_rot_x_.init(ros::NodeHandle(nh_cartesian, "pid_rot"));
	pid_controller_rot_x_.reset();
	pid_controller_rot_y_.init(ros::NodeHandle(nh_cartesian, "pid_rot"));
	pid_controller_rot_y_.reset();
	pid_controller_rot_z_.init(ros::NodeHandle(nh_cartesian, "pid_rot"));
	pid_controller_rot_z_.reset();
	
	
	start_server_ = nh_.advertiseService("start_tracking", &CobFrameTracker::start_tracking_cb, this);
	stop_server_ = nh_.advertiseService("stop_tracking", &CobFrameTracker::stop_tracking_cb, this);
	twist_pub_ = nh_twist.advertise<geometry_msgs::TwistStamped> ("command_twist_stamped", 1);
	
	tracking_frame_ = chain_tip_link_;
	tracking_ = false;
	
	ROS_INFO("...initialized!");
	return true;
}

void CobFrameTracker::run()
{
	ros::Time time = ros::Time::now();
	ros::Time last_update_time = time;
	ros::Duration period = time - last_update_time;
	
	ros::Rate r(update_rate_);
	while(ros::ok())
	{
		time = ros::Time::now();
		period = time - last_update_time;
		
		if(tracking_)
			publish_twist(period);
		
		last_update_time = time;
		
		ros::spinOnce();
		r.sleep();
	}
}

void CobFrameTracker::publish_twist(ros::Duration period)
{
	tf::StampedTransform transform_tf;
	geometry_msgs::TwistStamped twist_msg;
	double roll, pitch, yaw;
	
	try{
		tf_listener_.lookupTransform(chain_tip_link_, tracking_frame_, ros::Time(0), transform_tf);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
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
		
		//transform_tf.getBasis().getRPY(roll, pitch, yaw);
		//twist_msg.twist.angular.x = pid_controller_rot_x_.computeCommand(roll, period);
		//twist_msg.twist.angular.y = pid_controller_rot_y_.computeCommand(pitch, period);
		//twist_msg.twist.angular.z = pid_controller_rot_z_.computeCommand(yaw, period);
		
		twist_msg.twist.angular.x = pid_controller_rot_x_.computeCommand(transform_tf.getRotation().x(), period);
		twist_msg.twist.angular.y = pid_controller_rot_y_.computeCommand(transform_tf.getRotation().y(), period);
		twist_msg.twist.angular.z = pid_controller_rot_z_.computeCommand(transform_tf.getRotation().z(), period);
	}
	
	twist_msg.header.frame_id = chain_tip_link_;
	
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
	
	twist_pub_.publish(twist_msg);
}

bool CobFrameTracker::start_tracking_cb(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response)
{
	tracking_frame_ = request.data;
	tracking_ = true;
	
	response.success = true;
	
	return true;
}

bool CobFrameTracker::stop_tracking_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	tracking_frame_ = chain_tip_link_;
	tracking_ = false;
	
	//publish zero Twist for stopping
	geometry_msgs::TwistStamped twist_msg;
	twist_msg.header.frame_id = chain_tip_link_;
	twist_pub_.publish(twist_msg);
	
	return true;
}

