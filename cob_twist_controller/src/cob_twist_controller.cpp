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
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This package provides a generic Twist controller for the Care-O-bot
 *
 ****************************************************************/
#include <ros/ros.h>

#include <cob_twist_controller/cob_twist_controller.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>


bool CobTwistController::initialize()
{
	// JointNames
	if(!nh_.getParam("joint_names", joints_))
	{
		ROS_ERROR("Parameter 'joint_names' not set");
		return false;
	}
	dof_ = joints_.size();
	
	// Chain
	if(!nh_.getParam("base_link", chain_base_))
	{
		ROS_ERROR("Parameter 'base_link' not set");
		return false;
	}
	if (!nh_.getParam("tip_link", chain_tip_))
	{
		ROS_ERROR("Parameter 'tip_link' not set");
		return false;
	}
	
	// Cartesian VelLimits
	///Currently not used...
	///Shall we use these parameters here to reject Twists that are too high???
	if (!nh_.getParam("max_vel_lin", max_vel_lin_))
	{
		max_vel_lin_ = 10.0;	//m/sec
		ROS_WARN_STREAM("Parameter 'max_vel_lin' not set. Using default: " << max_vel_lin_);
	}
	if (!nh_.getParam("max_vel_rot", max_vel_rot_))
	{
		max_vel_rot_ = 6.28;	//rad/sec
		ROS_WARN_STREAM("Parameter 'max_vel_rot' not set. Using default: " << max_vel_rot_);
	}
	
	// SyncMM
	if (!nh_.getParam("base_compensation", base_compensation_))
	{
		base_compensation_ = false;
		ROS_WARN_STREAM("Base compensation disabled!");
	}
	
	if (!nh_.getParam("base_active", base_active_))
	{
		base_active_ = false;
		ROS_WARN_STREAM("Parameter 'base_active' not set. Base disabled!");
	}
	
	if (base_active_ && base_compensation_)
	{
		ROS_WARN("base_active and base_compensation cannot be enabled at the same time");
		return false;
	}
	
	// AugmentedSolverParams
	///ToDo: Read from ParameterServer
	AugmentedSolverParams params;
	params.damping_method = 5; //TRUNCATION
	params.eps = 0.001;
	params.damping_factor = 0.01;
	params.lambda0 = 0.05;
	params.wt = 0.0005;
	params.deltaRMax = 0.05;
	params.base_compensation = base_compensation_;
	params.base_active = base_active_;
	params.base_ratio = 0.0;
	
	//convert to reconfigure type
	cob_twist_controller::TwistControllerConfig config;
	config.damping_method = params.damping_method;
	config.eps = params.eps;
	config.damping_factor = params.damping_factor;
	config.lambda0 = params.lambda0;
	config.wt = params.wt;
	config.deltaRMax = params.deltaRMax;
	config.base_compensation = config.base_compensation;
	config.base_active = config.base_active;
	config.base_ratio = config.base_ratio;
	
	///parse robot_description and generate KDL chains
	KDL::Tree my_tree;
	std::string robot_desc_string;
	nh_.param("/robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}
	my_tree.getChain(chain_base_, chain_tip_, chain_);
	if(chain_.getNrOfJoints() == 0)
	{
		ROS_ERROR("Failed to initialize kinematic chain");
		return false;
	}
	
	///parse robot_description and set velocity limits
	urdf::Model model;
	if (!model.initParam("/robot_description"))
	{
		ROS_ERROR("Failed to parse urdf file for JointLimits");
		return false;
	}
	
	for(unsigned int i=0; i<dof_; i++)
	{
		limits_vel_.push_back(model.getJoint(joints_[i])->limits->velocity);
	}
	
	///initialize configuration control solver
	p_fksolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);	//used for debugging
	//p_iksolver_vel_ = new KDL::ChainIkSolverVel_pinv(chain_, 0.001, 5);
	p_augmented_solver_ = new augmented_solver(chain_, 0.001, 5);
	p_augmented_solver_->SetAugmentedSolverParams(params);
	
	///Setting up dynamic_reconfigure server for the AugmentedSolverParams
	reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_twist_controller::TwistControllerConfig>(reconfig_mutex_, nh_));
	reconfigure_server_->setCallback(boost::bind(&CobTwistController::reconfigure_callback,   this, _1, _2));
	
	///initialize variables and current joint values and velocities
	last_q_ = KDL::JntArray(chain_.getNrOfJoints());
	last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());
	
	///give tf_listener some time to fill tf-cache
	ros::Duration(2.0).sleep();
	
	///initialize ROS interfaces
	jointstate_sub = nh_.subscribe("/joint_states", 1, &CobTwistController::jointstate_cb, this);
	odometry_sub = nh_.subscribe("/base_controller/odometry", 1, &CobTwistController::odometry_cb, this);
	twist_sub = nh_.subscribe("command_twist", 1, &CobTwistController::twist_cb, this);
	twist_stamped_sub = nh_.subscribe("command_twist_stamped", 1, &CobTwistController::twist_stamped_cb, this);
	vel_pub = nh_.advertise<brics_actuator::JointVelocities>("command_vel", 1);
	//base_vel_pub = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
	base_vel_pub = nh_.advertise<geometry_msgs::Twist>("/base_controller/command_direct", 1);
	twist_pub_ = nh_.advertise<geometry_msgs::Twist> ("command_twist_normalized", 1);
	twist_real_pub_ = nh_.advertise<geometry_msgs::Twist> ("current_twist", 1);
	
	
	ROS_INFO("...initialized!");
	return true;
}

void CobTwistController::reconfigure_callback(cob_twist_controller::TwistControllerConfig &config, uint32_t level)
{
	AugmentedSolverParams params;
	params.damping_method = config.damping_method;
	params.eps = config.eps;
	params.damping_factor = config.damping_factor;
	params.lambda0 = config.lambda0;
	params.wt = config.wt;
	params.deltaRMax = config.deltaRMax;
	
	params.base_compensation = config.base_compensation;
	params.base_active = config.base_active;
	params.base_ratio = config.base_ratio;
	
	base_compensation_ = config.base_compensation;
	base_active_ = config.base_active;
	
	p_augmented_solver_->SetAugmentedSolverParams(params);
}



void CobTwistController::run()
{
	ROS_INFO("cob_twist_controller...spinning");
	ros::spin();
}


/// Orientation of twist_stamped_msg is with respect to coordinate system given in header.frame_id
void CobTwistController::twist_stamped_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	tf::StampedTransform transform_tf;
	KDL::Frame frame;
	KDL::Twist twist, twist_transformed;
	
	try{
		tf_listener_.lookupTransform(chain_base_, msg->header.frame_id, ros::Time(0), transform_tf);
		//frame.p = KDL::Vector(0.0, 0.0, 0.0);
		frame.p = KDL::Vector(transform_tf.getOrigin().x(), transform_tf.getOrigin().y(), transform_tf.getOrigin().z());
		frame.M = KDL::Rotation::Quaternion(transform_tf.getRotation().x(), transform_tf.getRotation().y(), transform_tf.getRotation().z(), transform_tf.getRotation().w());
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	
	tf::twistMsgToKDL(msg->twist, twist);
	twist_transformed = frame*twist;
	
	//ROS_DEBUG("Twist Vel (%f, %f, %f)", twist.vel.x(), twist.vel.y(), twist.vel.z());
	//ROS_DEBUG("Twist Rot (%f, %f, %f)", twist.rot.x(), twist.rot.y(), twist.rot.z());
	//ROS_DEBUG("TwistTransformed Vel (%f, %f, %f)", twist_transformed.vel.x(), twist_transformed.vel.y(), twist_transformed.vel.z());
	//ROS_DEBUG("TwistTransformed Rot (%f, %f, %f)", twist_transformed.rot.x(), twist_transformed.rot.y(), twist_transformed.rot.z());
	
	solve_twist(twist_transformed);
}

/// Orientation of twist_msg is with respect to chain_base coordinate system
void CobTwistController::twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
	KDL::Twist twist;
	tf::twistMsgToKDL(*msg, twist);
	
	solve_twist(twist);
}

/// Orientation of twist is with respect to chain_base coordinate system
void CobTwistController::solve_twist(KDL::Twist twist)
{
	//// calculate Cartesian velocity
	//// last resort for rejection in case it's too high
	//double vel_lin = std::sqrt(std::pow(twist.vel.x(), 2) + std::pow(twist.vel.y(), 2) + std::pow(twist.vel.z(), 2));
	//double vel_rot = std::sqrt(std::pow(twist.rot.x(), 2) + std::pow(twist.rot.y(), 2) + std::pow(twist.rot.z(), 2));
	//ROS_INFO_STREAM("INPUT: NormTwistLin: " << vel_lin << "; NormTwistRot: " << vel_rot);
	
	KDL::JntArray q_dot_ik(chain_.getNrOfJoints());
	
	if(base_active_)
	{
		q_dot_ik.resize(chain_.getNrOfJoints()+3);
	}
	
	if(base_compensation_)
	{
		//ROS_INFO("TwistIn Vel (%f, %f, %f)", twist.vel.x(), twist.vel.y(), twist.vel.z());
		//ROS_INFO("TwistIn Rot (%f, %f, %f)", twist.rot.x(), twist.rot.y(), twist.rot.z());
		
		twist = twist - twist_odometry_;
		
		//ROS_INFO("TwistOdometry Vel (%f, %f, %f)", twist_odometry_.vel.x(), twist_odometry_.vel.y(), twist_odometry_.vel.z());
		//ROS_INFO("TwistOdometry Rot (%f, %f, %f)", twist_odometry_.rot.x(), twist_odometry_.rot.y(), twist_odometry_.rot.z());
		//ROS_INFO("TwistDesired Vel (%f, %f, %f)", twist.vel.x(), twist.vel.y(), twist.vel.z());
		//ROS_INFO("TwistDesired Rot (%f, %f, %f)", twist.rot.x(), twist.rot.y(), twist.rot.z());
	}
	
	
	
	//int ret_ik = p_iksolver_vel_->CartToJnt(last_q_, twist, q_dot_ik);
	int ret_ik = p_augmented_solver_->CartToJnt(last_q_, twist, q_dot_ik);
	
	if(ret_ik < 0)
	{
		ROS_ERROR("No Vel-IK found!");
	}
	else
	{
		///normalize guarantees that velocities are within limits --- only needed for CartToJnt without damping
		//q_dot_ik = normalize_velocities(q_dot_ik);
		
		brics_actuator::JointVelocities vel_msg;
		vel_msg.velocities.resize(joints_.size());
		for(unsigned int i=0; i<dof_; i++)
		{
			vel_msg.velocities[i].joint_uri = joints_[i].c_str();
			vel_msg.velocities[i].unit = "rad";
			vel_msg.velocities[i].value = q_dot_ik(i);
			
			//ROS_WARN("DesiredVel %d: %f", i, q_dot_ik(i));
		}
		if(base_active_)
		{
			geometry_msgs::Twist base_vel_msg;
			base_vel_msg.linear.x = q_dot_ik(dof_);
			base_vel_msg.linear.y = q_dot_ik(dof_+1);
			base_vel_msg.angular.z = q_dot_ik(dof_+2);
			
			base_vel_pub.publish(base_vel_msg);
		}
		vel_pub.publish(vel_msg);
		
		
		/////---------------------------------------------------------------------
		///// Normalized q_dot into Twist
		//KDL::FrameVel FrameVel;
		//geometry_msgs::Twist twist_msg;
		//KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(last_q_,q_dot_ik);
		
		//int ret = p_fksolver_vel_->JntToCart(jntArrayVel,FrameVel,-1);
		
		//if(ret>=0)
		//{
			//KDL::Twist twist = FrameVel.GetTwist();
			//tf::twistKDLToMsg(twist,twist_msg);
		//}
		//twist_pub_.publish(twist_msg);
		/////---------------------------------------------------------------------
	}
}


void CobTwistController::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
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
		//ROS_DEBUG("Done Parsing");
		last_q_ = q_temp;
		last_q_dot_ = q_dot_temp;
		
		///---------------------------------------------------------------------
		/// current twist
		KDL::FrameVel FrameVel;
		//geometry_msgs::Twist twist_msg;
		KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(last_q_, last_q_dot_);
		
		int ret = p_fksolver_vel_->JntToCart(jntArrayVel, FrameVel, -1);
		
		if(ret>=0)
		{
			KDL::Twist twist = FrameVel.GetTwist();
			//tf::twistKDLToMsg(twist,twist_msg);
			//twist_real_pub_.publish(twist_msg);
			
			//ROS_INFO("TwistReal Vel (%f, %f, %f)", twist.vel.x(), twist.vel.y(), twist.vel.z());
			//ROS_INFO("TwistReal Rot (%f, %f, %f)", twist.rot.x(), twist.rot.y(), twist.rot.z());
		}
		///---------------------------------------------------------------------
	}
}


///ToDo: Something is still not correct with the rotational Twist transformation!!!
void CobTwistController::odometry_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::StampedTransform transform_tf;
	KDL::Frame frame;
	KDL::Twist twist_odometry, twist_odometry_transformed;
	
	try{
		//tf_listener_.lookupTransform(chain_base_, "odom_combined", ros::Time(0), transform_tf);
		//frame.p = KDL::Vector(0.0, 0.0, 0.0);
		tf_listener_.lookupTransform("base_footprint", chain_base_, ros::Time(0), transform_tf);
		frame.p = KDL::Vector(transform_tf.getOrigin().x(), transform_tf.getOrigin().y(), transform_tf.getOrigin().z());
		frame.M = KDL::Rotation::Quaternion(transform_tf.getRotation().x(), transform_tf.getRotation().y(), transform_tf.getRotation().z(), transform_tf.getRotation().w());
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	
	tf::twistMsgToKDL(msg->twist.twist, twist_odometry);
	twist_odometry_transformed = frame*twist_odometry;
	
	//ROS_INFO("TwistOdometry Vel (%f, %f, %f)", twist_odometry.vel.x(), twist_odometry.vel.y(), twist_odometry.vel.z());
	//ROS_INFO("TwistOdometry Rot (%f, %f, %f)", twist_odometry.rot.x(), twist_odometry.rot.y(), twist_odometry.rot.z());
	//ROS_INFO("TwistOdometryTransformed Vel (%f, %f, %f)", twist_odometry_transformed.vel.x(), twist_odometry_transformed.vel.y(), twist_odometry_transformed.vel.z());
	//ROS_INFO("TwistOdometryTransformed Rot (%f, %f, %f)", twist_odometry_transformed.rot.x(), twist_odometry_transformed.rot.y(), twist_odometry_transformed.rot.z());
	
	twist_odometry_ = twist_odometry_transformed;
}





KDL::JntArray CobTwistController::normalize_velocities(KDL::JntArray q_dot_ik)
{
	KDL::JntArray q_dot_norm = q_dot_ik;
	double max_factor = 1;
	for(unsigned int i=0; i<dof_; i++)
	{
		if(max_factor < std::fabs((q_dot_ik(i)/limits_vel_[i])))
		{
			max_factor = std::fabs((q_dot_ik(i)/limits_vel_[i]));
			ROS_WARN("Joint %d exceeds limit: Desired %f, Limit %f, Factor %f", i, q_dot_ik(i), limits_vel_[i], max_factor);
		}
	}
	
	///Do we need this?
	//if(base_active_)
	//{
		////TEST: limit base_velocities
		//double max_trans_velocity = 0.2;
		//double max_rot_velocity = 0.2;
		//if(max_factor < std::fabs((q_dot_ik(dof_)/max_trans_velocity)))
		//{
			//max_factor = std::fabs((q_dot_ik(dof_)/max_trans_velocity));
			//ROS_WARN("BaseTransX exceeds limit: Desired %f, Limit %f, Factor %f", q_dot_ik(dof_), max_trans_velocity, max_factor);
		//}
		//if(max_factor < std::fabs((q_dot_ik(dof_+1)/max_trans_velocity)))
		//{
			//max_factor = std::fabs((q_dot_ik(dof_+1)/max_trans_velocity));
			//ROS_WARN("BaseTransY exceeds limit: Desired %f, Limit %f, Factor %f", q_dot_ik(dof_+1), max_trans_velocity, max_factor);
		//}
		//if(max_factor < std::fabs((q_dot_ik(dof_+2)/max_rot_velocity)))
		//{
			//max_factor = std::fabs((q_dot_ik(dof_+2)/max_rot_velocity));
			//ROS_WARN("BaseRotZ exceeds limit: Desired %f, Limit %f, Factor %f", q_dot_ik(dof_+2), max_rot_velocity, max_factor);
		//}
	//}
	
	if(max_factor > 1)
	{
		ROS_INFO("Normalizing velocities!");
		for(unsigned int i=0; i<dof_; i++)
		{
			q_dot_norm(i) = q_dot_ik(i)/max_factor;
			ROS_WARN("Joint %d Normalized %f", i, q_dot_norm(i));
		}
	}
	
	return q_dot_norm;
}




