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
	ros::NodeHandle nh_action("frame_tracker");

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
	
	if (nh_cartesian.hasParam("chain_base_link"))
	{
		nh_cartesian.getParam("chain_base_link", chain_base_);
	}
	else
	{
		ROS_ERROR("No chain_base_link specified. Aborting!");
		return false;
	}

	if (nh_cartesian.hasParam("chain_tip_link"))
	{
		nh_cartesian.getParam("chain_tip_link", chain_tip_link_);
	}
	else
	{
		ROS_ERROR("No chain_tip_link specified. Aborting!");
		return false;
	}
	
	
	
	///parse robot_description and generate KDL chains
	nh_.param("/robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	if(!nh_.getParam("joint_names", joints_))
	{
		ROS_ERROR("Parameter 'joint_names' not set");
		return false;
	}
	dof_ = joints_.size();

	my_tree.getChain(chain_base_, chain_tip_link_, chain_);
	if(chain_.getNrOfJoints() == 0)
	{
		ROS_ERROR("Failed to initialize kinematic chain");
		return false;
	}

	///initialize ROS interfaces
	jointstate_sub = nh_.subscribe("/joint_states", 1, &CobFrameTracker::jointstate_cb, this);

	//initialize variables and current joint values and velocities
	ROS_INFO("chain_.getNrOfJoints() = %u", chain_.getNrOfJoints());
	last_q_ = KDL::JntArray(chain_.getNrOfJoints());
	last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());
	q_temp = last_q_;
	q_dot_temp = last_q_dot_;
	
	
	
	
	
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
	
	pid_controller_rot_x_.init(ros::NodeHandle(nh_cartesian, "pid_rot_x"));
	pid_controller_rot_x_.reset();
	pid_controller_rot_y_.init(ros::NodeHandle(nh_cartesian, "pid_rot_y"));
	pid_controller_rot_y_.reset();
	pid_controller_rot_z_.init(ros::NodeHandle(nh_cartesian, "pid_rot_z"));
	pid_controller_rot_z_.reset();
	
	
	start_server_ = nh_.advertiseService("start_tracking", &CobFrameTracker::start_tracking_cb, this);
	stop_server_ = nh_.advertiseService("stop_tracking", &CobFrameTracker::stop_tracking_cb, this);
	twist_pub_ = nh_twist.advertise<geometry_msgs::TwistStamped> ("command_twist_stamped", 1);
	
	tracking_frame_ = chain_tip_link_;
	tracking_ = false;

	reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_frame_tracker::FrameTrackerConfig>(reconfig_mutex_, nh_action));
	reconfigure_server_->setCallback(boost::bind(&CobFrameTracker::reconfigure_callback,   this, _1, _2));

	cart_distance_ = 0.0;
	rot_distance_ = 0.0;
	abortion_message_ = "--- NO INTERVENTION ---";
	tracking_goal_ = false;
	
	//ABORTION CRITERIA:
	current_twist_.Zero();
	target_twist_.Zero();

	value_failed_ = false;
	numberOfFailedValues_ = 0;

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
		double maxdurationOftracking = start_of_tracking_.toSec() + double(tracking_time_);
		if(tracking_) {
			// tracking on goal or tracking because of service call
			if (tracking_goal_) {
				if (tracking_time_ != 0) {
					ROS_INFO("Server is waiting for trackingtime = %f to end.", maxdurationOftracking-time.toSec());
					if (time.toSec() > maxdurationOftracking) {
						tracking_ = false;
						CobFrameTracker::succeed();
					}
				}
				// target reaches active frame --> successful
				if (stop_on_goal_) {
					ROS_INFO("Goal was reached!");
					result_.success = true;
					CobFrameTracker::succeed();
				}

				//--> ABORTION CRITERIA ONLY HERE
				if (CobFrameTracker::searchForAbortionCriteria()) {
					CobFrameTracker::abort();
				}
			}
////			ROS_INFO("show abortion message [%s]", abortion_message_.c_str());
//			//--> ABORTION CRITERIA ONLY HERE FOR DEBUGGING! DELETE AFTERWARDS!
//			if (CobFrameTracker::searchForAbortionCriteria()) {
////				CobFrameTracker::abort();
//			}

			publish_twist(period);
		}
//		else
//		{
//			ROS_INFO("[Server is waiting for a new goal!]");
//		}
		
		last_update_time = time;
		
		ros::spinOnce();
		r.sleep();

		if (as_.isActive()) {
//			ROS_INFO("Sending feedback:");
			as_.publishFeedback(feedback_);
		}
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

	//eukl distance:
	cart_distance_ = sqrt(pow(transform_tf.getOrigin().x(),2) + pow(transform_tf.getOrigin().y(),2) + pow(transform_tf.getOrigin().z(),2));
	//rot distance:
	// TODO: change to cartesian rot
//	rot_distance_ = 2* acos(transform_msg.transform.rotation.w);
	//get target_twist
	target_twist_.vel.x(twist_msg.twist.linear.x);
	target_twist_.vel.y(twist_msg.twist.linear.y);
	target_twist_.vel.z(twist_msg.twist.linear.z);
	target_twist_.rot.x(twist_msg.twist.angular.x);
	target_twist_.rot.y(twist_msg.twist.angular.y);
	target_twist_.rot.z(twist_msg.twist.angular.z);

	twist_pub_.publish(twist_msg);
}

bool CobFrameTracker::start_tracking_cb(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response)
{
	if (tracking_) {
		ROS_INFO("CobFrameTracker start was denied! FrameTracker is already tracking a goal");
		response.success = false;
		response.message = "FrameTracker is already tracking goal!";
		return false;
	}
	else
	{
		ROS_INFO("CobFrameTracker started WITHOUT SECURITY MONITORING");
		tracking_frame_ = request.data;
		tracking_ = true;
		tracking_goal_ = false;
		response.success = true;
		return true;
	}
}

bool CobFrameTracker::stop_tracking_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (tracking_) {
		if (tracking_goal_) {
			ROS_INFO("CobFrameTracker stop was denied because TrackingAction is tracking a goal. You must send 'cancel goal' to the action server instead.");
			return false;
		}
		ROS_INFO("CobFrameTracker stopped successfully");
		tracking_frame_ = chain_tip_link_;
		tracking_ = false;
		//publish zero Twist for stopping
		geometry_msgs::TwistStamped twist_msg;
		twist_msg.header.frame_id = chain_tip_link_;
		twist_pub_.publish(twist_msg);
		return true;
	}
	else
	{
		ROS_INFO("CobFrameTracker stop denied because nothing was tracked.");
		return false;
	}
}

void CobFrameTracker::goalCB()
{
	ROS_INFO("this is a new goal callback");
	ROS_INFO("Waiting for a new goal");
	if (as_.isNewGoalAvailable()) {
		boost::shared_ptr<const cob_frame_tracker::FrameTrackingGoal> goal_= as_.acceptNewGoal();
		tracking_frame_ = goal_->tracking_frame;
		stop_on_goal_ = goal_->stop_on_goal;
		tracking_time_ = goal_->tracking_time;
		tracking_ = true;
		tracking_goal_ = true;
		start_of_tracking_ = ros::Time::now();
//		ROS_INFO("Output: Received stop_on_goal_ = %s", stop_on_goal_ ? "true" : "false" );
//		ROS_INFO("Output: Received tracking_time_ = %u", tracking_time_);
	}
}

void CobFrameTracker::preemptCB()
{
	ROS_INFO("this is the preempt callback");
	as_.setPreempted(result_);
	tracking_ = false;
	tracking_goal_ = false;
//	ROS_INFO("active_frame = %s", tracking_frame_.c_str());
	tracking_frame_ = chain_tip_link_;
	geometry_msgs::TwistStamped twist_msg;
	twist_msg.header.frame_id = chain_tip_link_;
	twist_pub_.publish(twist_msg);
}

void CobFrameTracker::succeed()
{
	ROS_INFO("Goal succeeded!");
	as_.setSucceeded(result_, "succeeded text");
	tracking_ = false;
	tracking_goal_ = false;
	geometry_msgs::TwistStamped twist_msg;
	twist_msg.header.frame_id = chain_tip_link_;
	twist_pub_.publish(twist_msg);
}

void CobFrameTracker::abort()
{
	ROS_INFO("this is the abort method");
	result_.success = false;
	as_.setAborted(result_, abortion_message_);
	ROS_WARN("Tracking has been aborted because of %s", abortion_message_.c_str());
	tracking_ = false;
	tracking_goal_ = false;
	tracking_frame_ = chain_tip_link_;
	geometry_msgs::TwistStamped twist_msg;
	twist_msg.header.frame_id = chain_tip_link_;
	twist_pub_.publish(twist_msg);
}

bool CobFrameTracker::searchForAbortionCriteria()
{
	KDL::Twist current_twist_local_(current_twist_);
	KDL::Twist target_twist_local_(target_twist_);

	bool noTwist = CobFrameTracker::checkNoTwistErrors(current_twist_local_);
	bool distance = CobFrameTracker::checkDistance(cart_distance_, 0.0);
	bool deadlock = false;

//	ROS_INFO("distance value %f", cart_distance_);
//	ROS_INFO("rotdistance value %f", rot_distance_);
//	ROS_INFO("distance is %s", distance ? "true" : "false");
//	ROS_INFO("deadlock is %s", noTwist ? "true" : "false");

	if ((distance && noTwist) || (!distance && !noTwist))
	{
		deadlock = true;
//		return true;
	}
	else
	{
		deadlock = false;
	}

	bool value = CobFrameTracker::checkDeviationErrors(current_twist_local_, target_twist_local_);

//	ROS_INFO("value is %s", value ? "true" : "false");

	if (numberOfFailedValues_ < 50) {
		numberOfFailedValues_++;
		if (!value) {
			abortion_message_ = "--- NO INTERVENTION ---";
			value_failed_ = false;
		}
	}
	else
	{
		numberOfFailedValues_ = 0;
		if (value_failed_) {
			abortion_message_ = "Devitation between target and current twist!";
			return true;
		}
		if (value){
			value_failed_ = true;
		}
	}
	return false;
}

void CobFrameTracker::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
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
		jntToCartSolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);
		int ret = jntToCartSolver_vel_->JntToCart(jntArrayVel,FrameVel,-1);
		if(ret>=0)
		{
			KDL::Twist twist = FrameVel.GetTwist();
			current_twist_ = twist;
		}
		else
		{
			ROS_WARN("ChainFkSolverVel failed!");
		}
		///---------------------------------------------------------------------
	}
}

void CobFrameTracker::reconfigure_callback(cob_frame_tracker::FrameTrackerConfig &config, uint32_t level)
{
	cart_min_dist_threshold_lin_ = config.cart_min_dist_threshold_lin;
	cart_min_dist_threshold_rot_ = config.cart_min_dist_threshold_rot;
	twist_dead_threshold_lin_ = config.twist_dead_threshold_lin;
	twist_dead_threshold_rot_ = config.twist_dead_threshold_rot;
	twist_deviation_threshold_lin_ = config.twist_deviation_threshold_lin;
	twist_deviation_threshold_rot_ = config.twist_deviation_threshold_rot;
}

bool CobFrameTracker::checkDistance(const double dist, const double rot)
{
	if (dist > cart_min_dist_threshold_lin_)
	{
		return true;
	}
//	if (rot > cart_min_dist_threshold_rot_)
//	{
//		return true;
//	}
	return false;
}

bool CobFrameTracker::checkNoTwistErrors(const KDL::Twist current)
{
	KDL::Twist cur(current);
	bool ndead[6] =  {false,false,false,false,false,false};
	if (abs(cur.vel.x() > twist_dead_threshold_lin_))
	{
		ndead[0] = true;
	}
	if (abs(cur.vel.y() > twist_dead_threshold_lin_))
	{
		ndead[1] = true;
	}
	if (abs(cur.vel.z() > twist_dead_threshold_lin_))
	{
		ndead[2] = true;
	}
	if (abs(cur.rot.x() > twist_dead_threshold_rot_))
	{
		ndead[3] = true;
	}
	if (abs(cur.rot.x() > twist_dead_threshold_rot_))
	{
		ndead[4] = true;
	}
	if (abs(cur.rot.x() > twist_dead_threshold_rot_))
	{
		ndead[5] = true;
	}
//	ROS_INFO("NDead = [%s, %s, %s, %s, %s, %s]", ndead[0] ? "true" : "false",  ndead[1] ? "true" : "false", ndead[2] ? "true" : "false", ndead[3] ? "true" : "false", ndead[4] ? "true" : "false", ndead[5] ? "true" : "false");
	for (int i = 0; i < 6 ; i++) {
		if (ndead[i]) {
			return true;
		}
	}
	return false;
}

bool CobFrameTracker::checkDeviationErrors(const KDL::Twist current, const KDL::Twist target)
{
	KDL::Twist cur(current);
	KDL::Twist tar(target);
	bool deviations[6] = {false, false, false, false, false, false};
//	ROS_INFO("current_value_lin = [%f,%f,%f]", cur.vel.x(), cur.vel.y(), cur.vel.z());
//	ROS_INFO("target_value_lin = [%f,%f,%f]", tar.vel.x(), tar.vel.y(), tar.vel.z());
//	ROS_INFO("current_value_rot = [%f,%f,%f]", cur.rot.x(), cur.rot.y(), cur.rot.z());
//	ROS_INFO("target_value_rot = [%f,%f,%f]", tar.rot.x(), tar.rot.y(), tar.rot.z());
	if (cur.vel.x() < (target.vel.x() - twist_deviation_threshold_lin_) || (target.vel.x() + twist_deviation_threshold_lin_) < cur.vel.x())
	{
		deviations[0] = true;
	}
	if (cur.vel.y() < (target.vel.y() - twist_deviation_threshold_lin_) || (target.vel.y() + twist_deviation_threshold_lin_) < cur.vel.y())
	{
		deviations[1] = true;
	}
	if (cur.vel.z() < (target.vel.z() - twist_deviation_threshold_lin_) || (target.vel.z() + twist_deviation_threshold_lin_) < cur.vel.z())
	{
		deviations[2] = true;
	}
	if (cur.rot.x() < (target.rot.x() - twist_deviation_threshold_rot_) || (target.rot.x() + twist_deviation_threshold_rot_) < cur.rot.x())
	{
		deviations[3] = true;
	}
	if (cur.rot.y() < (target.rot.y() - twist_deviation_threshold_rot_) || (target.rot.y() + twist_deviation_threshold_rot_) < cur.rot.y())
	{
		deviations[4] = true;
	}
	if (cur.rot.z() < (target.rot.z() - twist_deviation_threshold_rot_) || (target.rot.z() + twist_deviation_threshold_rot_) < cur.rot.z())
	{
		deviations[5] = true;
	}
//	ROS_INFO("Deviated twists = [%s, %s, %s, %s, %s, %s]", deviations[0] ? "true" : "false",  deviations[1] ? "true" : "false", deviations[2] ? "true" : "false", deviations[3] ? "true" : "false", deviations[4] ? "true" : "false", deviations[5] ? "true" : "false");
	for (int i = 0; i < 6 ; i++) {
		if (deviations[i]) {
			return true;
		}
	}
	return false;
}
