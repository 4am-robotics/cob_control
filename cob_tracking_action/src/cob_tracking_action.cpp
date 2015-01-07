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
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <cob_tracking_action/cob_tracking_action.h>
#include <cob_tracking_action/TrackingAction.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <math.h>

bool TrackingAction::initialize()
{
	ros::NodeHandle nh_action("tracking_action");

	//TargetXDotReader insert on BEGIN:

	ros::NodeHandle nh_cartesian("cartesian_controller");
	ros::NodeHandle nh_base("base");

	///parse robot_description and generate KDL chains
	nh_.param("/robot_description", robot_desc_string, std::string());
	//ROS_INFO("%s",robot_desc_string.c_str());
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

	// Chain
	if(!nh_cartesian.getParam("base_link", chain_base_))
	{
		ROS_ERROR("Parameter 'base_link' not set");
		return false;
	}
	if (!nh_cartesian.getParam("tip_link", chain_tip_))
	{
		ROS_ERROR("Parameter 'tip_link' not set");
		return false;
	}

	my_tree.getChain(chain_base_, chain_tip_, chain_);
	if(chain_.getNrOfJoints() == 0)
	{
		ROS_ERROR("Failed to initialize kinematic chain");
		return false;
	}

	///initialize ROS interfaces
	jointstate_sub = nh_.subscribe("/joint_states", 1, &TrackingAction::jointstate_cb, this);


	//initialize variables and current joint values and velocities
	ROS_INFO("chain_.getNrOfJoints() = %u", chain_.getNrOfJoints());
	last_q_ = KDL::JntArray(chain_.getNrOfJoints());
	last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());
	q_temp = last_q_;
	q_dot_temp = last_q_dot_;
	//readXDotTarget insert on END:


	//FrameTrackingInsert:float64’ is not a member of ‘std_msgs’
	ros::NodeHandle nh_twist("twist_controller");
	//ros::NodeHandle nh_cartesian("cartesian_controller");

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
	debug_pub_1 = nh_action.advertise<std_msgs::Float64> ("cart_distance", 1);
	debug_pub_2 = nh_action.advertise<std_msgs::Float64> ("rot_distance", 1);
	debug_pub_3 = nh_action.advertise<std_msgs::Float64> ("cart_twist", 1);
	debug_pub_4 = nh_action.advertise<std_msgs::Float64> ("rot_twist", 1);
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
	KDL::Vector vel(0.0,0.0,0.0);
	KDL::Vector rot(0.0,0.0,0.0);
	current_twist_.vel = vel;
	current_twist_.rot = rot;
	//register the goal and feeback callbacks

	// JointNames
	//if(!nh_.getParam("joint_names", joints_))
	//{
	//	ROS_ERROR("Parameter 'joint_names' not set");
	//	return false;
	//}
	//dof_ = joints_.size();


	//convert to reconfigure type
	cob_tracking_action::TrackingActionConfig config;

	config.timer_value = 10;
	config.base_active = false;
	config.base_ratio = 0.0;
	config.distance_threshold = 0.005;

	reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_tracking_action::TrackingActionConfig>(reconfig_mutex_, nh_action));
	reconfigure_server_->setCallback(boost::bind(&TrackingAction::reconfigure_callback,   this, _1, _2));

	cart_distance_ = 0.0;
	rot_distance_ = 0.0;
	abortion_message_ = "";
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
			ROS_INFO("cart_distance_ = %f rot_distance_ = %f", cart_distance_, rot_distance_);
			ROS_INFO("current_twist [%f, %f, %f, %f, %f, %f]", current_twist_.vel.x(), current_twist_.vel.y(), current_twist_.vel.z(), current_twist_.rot.x(), current_twist_.rot.y(), current_twist_.rot.z());
			ROS_INFO("target_twist [%f, %f, %f, %f, %f, %f]", target_twist_.vel.x(), target_twist_.vel.y(), target_twist_.vel.z(), target_twist_.rot.x(), target_twist_.rot.y(), target_twist_.rot.z());
			ROS_INFO("diff_twist [%f, %f, %f, %f, %f, %f]", diff_twist_.vel.x(), diff_twist_.vel.y(), diff_twist_.vel.z(), diff_twist_.rot.x(), diff_twist_.rot.y(), diff_twist_.rot.z());
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
			//readXDotCurrent insert ON BEGIN:

//			ROS_INFO("X dot current output = [%f,%f,%f,%f,%f,%f]", vector_vel_.x, vector_vel_.y, vector_vel_.z, vector_rot_.x, vector_rot_.y, vector_rot_.z);
			//readXDotCurrent insert ON END:
			if (TrackingAction::searchForAbortionCriteria()) {
				TrackingAction::abort();
			}

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
	//eukl distance:
	cart_distance_ = sqrt(transform_msg.transform.translation.x * transform_msg.transform.translation.x + transform_msg.transform.translation.y * transform_msg.transform.translation.y + transform_msg.transform.translation.z * transform_msg.transform.translation.z);
	//rot distance:
	rot_distance_ = 2* acos(transform_msg.transform.rotation.w);

//	cart_distance_.y(sqrt(transform_msg.transform.rotation));
	//my calculations begin


//	eukl_dist_lin_ = sqrt(transform_msg.transform.translation.x * transform_msg.transform.translation.x + transform_msg.transform.translation.y * transform_msg.transform.translation.y + transform_msg.transform.translation.z * transform_msg.transform.translation.z);
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
	//get target_twist
	target_twist_.vel.x(twist_msg.linear.x);
	target_twist_.vel.y(twist_msg.linear.y);
	target_twist_.vel.z(twist_msg.linear.z);
	target_twist_.rot.x(twist_msg.angular.x);
	target_twist_.rot.y(twist_msg.angular.y);
	target_twist_.rot.z(twist_msg.angular.z);
	//publish
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
	result_.success = false;
	as_.setAborted(result_, abortion_message_);
	ROS_WARN("Tracking has been aborted because of %s", abortion_message_.c_str());
	tracking_ = false;
	target_tracking_frame_ = active_frame_;
	geometry_msgs::Twist twist_msg;
	twist_pub_.publish(twist_msg);
}

bool TrackingAction::searchForAbortionCriteria()
{
	diff_twist_.vel.x(target_twist_.vel.x() - current_twist_.vel.x());
	diff_twist_.vel.y(target_twist_.vel.y() - current_twist_.vel.y());
	diff_twist_.vel.z(target_twist_.vel.z() - current_twist_.vel.z());
	diff_twist_.rot.x(target_twist_.rot.x() - current_twist_.rot.x());
	diff_twist_.rot.y(target_twist_.rot.y() - current_twist_.rot.y());
	diff_twist_.rot.z(target_twist_.rot.z() - current_twist_.rot.z());

	bool deadlock;
	bool no_twist;
	double trans_twist_norm = diff_twist_.vel.Norm();
	double rot_twist_norm = diff_twist_.rot.Norm();
	ROS_INFO("trans_twist_norm, rot_twist_norm [%f, %f]", trans_twist_norm, rot_twist_norm);

	if (trans_twist_norm < distance_threshold_) {
		trans_twist_norm = 0;
	}

	if (rot_twist_norm < distance_threshold_) {
		rot_twist_norm = 0;
	}

	if (cart_distance_ < distance_threshold_) {
		cart_distance_ = 0;
	}

	if (rot_distance_ < distance_threshold_) {
		rot_distance_ = 0;
	}
	/// ABORTION CRITERIA:
	if (cart_distance_ == 0 && rot_distance_ == 0) {
		deadlock = true;
	}
	else
		deadlock = false;

	if (trans_twist_norm == 0 && rot_twist_norm == 0) {
		no_twist = true;
	}
	else
		no_twist = false;

	// x_current = x_target but diff_twist != 0;
	if (deadlock && !no_twist) {
		abortion_message_ = "ABORTION CRITERIA 1";
//		return true;
	}

	// x_current != x_target but diff_twist = 0;
	if (!deadlock && no_twist) {
		abortion_message_ = "ABORTION CRITERIA 2";
//		return true;
	}


	///current twist mean:
//	TrackingAction::updateList(current_twist_, 10);
//	ROS_INFO("list size %lu", list_of_twists_stamped_.size());
//	TrackingAction::calcMean();
//	for (int i=0; i < list_of_twists_stamped_.size(); i++) {
//		ROS_INFO("erstes element: %f", list_of_twists_stamped_.at(i).at(0));
//	}
	updateList2(trans_twist_norm, 5);
	double mean = meanNorm();
	ROS_INFO("mean trans twist is = %f", mean);

	std_msgs::Float64 msg;
	msg.data = cart_distance_;
	debug_pub_1.publish(msg);
	msg.data = rot_distance_;
	debug_pub_2.publish(msg);
	msg.data = trans_twist_norm;
	debug_pub_3.publish(msg);
	msg.data = rot_twist_norm;
	debug_pub_4.publish(msg);
	//X_target - X_current
	int x_diff = 0;
	///Case 1:
	///
	if (eukl_dist_lin_ < distance_threshold_) {
		x_diff = 0;
	}



	if (x_diff == 0) {
//		if (start_timer_) {
//			 timer = ros::Time::now();
//		}
//		///start a timer
	}
	else {
		///stop the timer
	}
	return false;
}

void TrackingAction::calcMean() {
	std::vector<double> mean_list;
	mean_list.push_back(0.0);
	mean_list.push_back(0.0);
	mean_list.push_back(0.0);
	mean_list.push_back(0.0);
	mean_list.push_back(0.0);
	mean_list.push_back(0.0);
	int number_of_elements = list_of_twists_stamped_.size();
	for (int j=0; j<6; j++) {
		for (int i=0; i<number_of_elements; i++) {
			mean_list.at(j) = mean_list.at(j) + list_of_twists_stamped_.at(i).at(j);
		}
		mean_list.at(j) = mean_list.at(j) / number_of_elements;
	}

	double norm = 0.0;
	for (int u = 0; u <6; u++) {
		norm = norm + sqrt(mean_list.at(u) * mean_list.at(u));
	}
	ROS_INFO("NORM OF MEAN = %f", norm);
}

void TrackingAction::updateList2(double norm, double time_filter_threshold) {
	std::vector<double> current_norm;
	double current_time = ros::Time::now().toSec();
	current_norm.push_back(norm);
	current_norm.push_back(current_time);
	list_of_norms_stamped_.push_back(current_norm);
	while(list_of_norms_stamped_.front().at(1) < current_time - time_filter_threshold) {
		list_of_norms_stamped_.erase(list_of_norms_stamped_.begin());
	}
}

double TrackingAction::meanNorm() {
	double mean = 0.0;
	for (int i = 0; i < list_of_norms_stamped_.size(); i++) {
		mean = mean + list_of_norms_stamped_.at(i).at(0);
	}
	mean = mean / list_of_norms_stamped_.size();
	return mean;
}

void TrackingAction::updateList(KDL::Twist twist,  double time_filter_threshold) {
	std::vector<double> current_twist_list_;
	double current_time = ros::Time::now().toSec();
	current_twist_list_.push_back(twist.vel.x());
	current_twist_list_.push_back(twist.vel.y());
	current_twist_list_.push_back(twist.vel.z());
	current_twist_list_.push_back(twist.rot.x());
	current_twist_list_.push_back(twist.rot.y());
	current_twist_list_.push_back(twist.rot.z());
	current_twist_list_.push_back(current_time);
	list_of_twists_stamped_.push_back(current_twist_list_);
	while(list_of_twists_stamped_.front().at(6) < current_time - time_filter_threshold ) {
		list_of_twists_stamped_.erase(list_of_twists_stamped_.begin());
	}
}

void TrackingAction::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
//	ROS_INFO("Callback has been called .. flag1");
	KDL::JntArray q_temp = last_q_;
	KDL::JntArray q_dot_temp = last_q_dot_;
	int count = 0;
//	ROS_INFO("Callback has been called .. flag2");
//	ROS_INFO("dof_ = %u",dof_);
	for(unsigned int j = 0; j < dof_; j++)
	{
//		ROS_INFO("name.size = %lu", msg->name.size());
		for(unsigned int i = 0; i < msg->name.size(); i++)
		{

			if(strcmp(msg->name[i].c_str(), joints_[j].c_str()) == 0)
			{
//				ROS_INFO("erfolgreich");
				q_temp(j) = msg->position[i];
				q_dot_temp(j) = msg->velocity[i];
				count++;
				break;
			}
		}
	}
//	ROS_INFO("Callback has been called .. flag3");
	if(count == joints_.size())
	{
//		ROS_INFO("joint_size = %lu",joints_.size());
//		ROS_INFO("Callback has been called .. flag4");
		//ROS_DEBUG("Done Parsing");
		last_q_ = q_temp;
		last_q_dot_ = q_dot_temp;

		///---------------------------------------------------------------------
		/// current twist
		KDL::FrameVel FrameVel;
		//geometry_msgs::Twist twist_msg;
		KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(last_q_, last_q_dot_);
//		ROS_INFO("Callback has been called .. flag5");
		jntToCartSolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);
//		ROS_INFO("Callback has beend called ... flag6");
		int ret = jntToCartSolver_vel_->JntToCart(jntArrayVel,FrameVel,-1);
//		int ret = p_fksolver_vel_->JntToCart(jntArrayVel, FrameVel, -1);
//		ROS_INFO("Callback has been called .. flag7");
		if(ret>=0)
		{
//			ROS_INFO("Callback has been called .. flag8");
			KDL::Twist twist = FrameVel.GetTwist();
//			ROS_INFO("Callback has been called .. flag9");
			//tf::twistKDLToMsg(twist,twist_msg);
			//twist_real_pub_.publish(twist_msg);
			current_twist_ = twist;
//			ROS_INFO("TwistReal Vel (%f, %f, %f)", twist.vel.x(), twist.vel.y(), twist.vel.z());
//			ROS_INFO("TwistReal Rot (%f, %f, %f)", twist.rot.x(), twist.rot.y(), twist.rot.z());
//			ROS_INFO("Callback has been called .. flag10");
		}
		else
		{
			ROS_WARN("ChainFkSolverVel failed!");
		}
		///---------------------------------------------------------------------
	}
}

void TrackingAction::reconfigure_callback(cob_tracking_action::TrackingActionConfig &config, uint32_t level)
{
	timer_value_ = config.timer_value;
	base_active_ = config.base_active;
	base_ratio_ = config.base_ratio;
	distance_threshold_ = config.distance_threshold;
}
