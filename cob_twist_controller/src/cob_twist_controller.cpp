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
#include <visualization_msgs/Marker.h>


bool CobTwistController::initialize()
{
	ros::NodeHandle nh_twist("twist_controller");
	ros::NodeHandle nh_cartesian("cartesian_controller");
	ros::NodeHandle nh_base("base");
	
	// JointNames
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
	
	// Cartesian VelLimits
	///Currently not used...
	///Shall we use these parameters here to reject Twists that are too high???
	if (!nh_cartesian.getParam("max_vel_lin", max_vel_lin_))
	{
		max_vel_lin_ = 10.0;	//m/sec
		ROS_WARN_STREAM("Parameter 'max_vel_lin' not set. Using default: " << max_vel_lin_);
	}
	if (!nh_cartesian.getParam("max_vel_rot", max_vel_rot_))
	{
		max_vel_rot_ = 6.28;	//rad/sec
		ROS_WARN_STREAM("Parameter 'max_vel_rot' not set. Using default: " << max_vel_rot_);
	}
	
	// SyncMM
	if (!nh_cartesian.getParam("base_compensation", base_compensation_))
	{
		base_compensation_ = false;
		ROS_WARN_STREAM("Base compensation disabled!");
	}
	
	if (!nh_cartesian.getParam("base_active", base_active_))
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
	config.JLA_active = config.JLA_active;
	config.enforce_limits = config.enforce_limits;
	config.reset_markers = config.reset_markers;
	config.base_min_vel = config.base_min_vel;
	
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
		limits_min_.push_back(model.getJoint(joints_[i])->limits->lower+0.2);
		limits_max_.push_back(model.getJoint(joints_[i])->limits->upper-0.2);
	}
	
	///initialize configuration control solver
	p_fksolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);	//used for debugging
	//p_iksolver_vel_ = new KDL::ChainIkSolverVel_pinv(chain_, 0.001, 5);
	p_augmented_solver_ = new augmented_solver(chain_, 0.001, 5);
	p_augmented_solver_->SetAugmentedSolverParams(params);
	
	///Setting up dynamic_reconfigure server for the AugmentedSolverParams
	reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_twist_controller::TwistControllerConfig>(reconfig_mutex_, nh_cartesian));
	reconfigure_server_->setCallback(boost::bind(&CobTwistController::reconfigure_callback,   this, _1, _2));
	
	///initialize variables and current joint values and velocities
	last_q_ = KDL::JntArray(chain_.getNrOfJoints());
	last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());
	
	///give tf_listener some time to fill tf-cache
	ros::Duration(2.0).sleep();
	
	///initialize ROS interfaces
	jointstate_sub = nh_.subscribe("/joint_states", 1, &CobTwistController::jointstate_cb, this);
	odometry_sub = nh_base.subscribe("controller/odometry", 1, &CobTwistController::odometry_cb, this);
	twist_sub = nh_twist.subscribe("command_twist", 1, &CobTwistController::twist_cb, this);
	twist_stamped_sub = nh_twist.subscribe("command_twist_stamped", 1, &CobTwistController::twist_stamped_cb, this);
	vel_pub = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);
	//base_vel_pub = nh_base.advertise<geometry_msgs::Twist>("controller/command", 1);
	base_vel_pub = nh_base.advertise<geometry_msgs::Twist>("controller/command_direct", 1);
	twist_pub_ = nh_twist.advertise<geometry_msgs::Twist> ("debug/twist_normalized", 1);
	twist_real_pub_ = nh_twist.advertise<geometry_msgs::Twist> ("debug/twist_mani", 1);
	twist_real_base_pub_ = nh_twist.advertise<geometry_msgs::Twist> ("debug/twist_base", 1);
	soll_twist_pub_ = nh_twist.advertise<geometry_msgs::Twist> ("debug/twist_ee", 1);
	pose_base_pub_ = nh_twist.advertise<geometry_msgs::Pose> ("debug/pose_base", 0);
	pose_tip_pub_ = nh_twist.advertise<geometry_msgs::Pose> ("debug/pose_tip", 0);
	vis_pub_ee_ = nh_twist.advertise<visualization_msgs::Marker>( "debug/vis_ee", 0 );
	vis_pub_base_ = nh_twist.advertise<visualization_msgs::Marker>( "debug/vis_base", 0 );

	ros::Time time_ = ros::Time::now();
	ros::Time last_update_time_ = time_;
	ros::Duration period_ = time_ - last_update_time_;
	
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
	params.JLA_active = config.JLA_active;
	params.enforce_limits = config.enforce_limits;
	reset_markers_ = config.reset_markers;
	base_compensation_ = config.base_compensation;
	base_active_ = config.base_active;
	params.base_min_vel = config.base_min_vel;
	
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
		//frame.p = KDL::Vector(transform_tf.getOrigin().x(), transform_tf.getOrigin().y(), transform_tf.getOrigin().z());
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

KDL::Twist CobTwistController::getBaseCompensatedTwist(KDL::Twist twist,KDL::Twist twist_odometry){
	return (twist-twist_odometry);
}

/// Orientation of twist is with respect to chain_base coordinate system
void CobTwistController::solve_twist(KDL::Twist twist)
{
	geometry_msgs::Twist tw;
	KDL::Twist tw_debug;
	KDL::Frame frame_tip,frame_base;
	tf::StampedTransform tf_tip,tf_base;
	geometry_msgs::Pose pose_tip,pose_base;
	geometry_msgs::Point point_base,point_ee;

	
	int ret_ik;
	KDL::Twist base_twist,twist_odom;
	KDL::Frame frame,frame2,frame3;
	tf::StampedTransform transform_tip_basefootprint,transform_base_basefootprint,tf_d;
	KDL::JntArray q_dot_ik(chain_.getNrOfJoints());
	KDL::JntArray q_dot_ik_debug(chain_.getNrOfJoints());
			///DEBUG
		try{
			tf_listener_.waitForTransform("odom_combined",chain_tip_, ros::Time(0), ros::Duration(0.5));
			tf_listener_.lookupTransform("odom_combined",chain_tip_, ros::Time(0), tf_tip);
		}catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			return;
		}
		try{
			tf_listener_.waitForTransform("odom_combined","base_link", ros::Time(0), ros::Duration(0.5));
			tf_listener_.lookupTransform("odom_combined","base_link", ros::Time(0), tf_base);
		}catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			return;
		}
		
		try{
			tf_listener_.waitForTransform("base_link",chain_base_, ros::Time(0), ros::Duration(0.5));
			tf_listener_.lookupTransform("base_link",chain_base_, ros::Time(0), tf_d);
		}catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			return;
		}
		
		frame_tip.p = KDL::Vector(tf_tip.getOrigin().x(), tf_tip.getOrigin().y(), tf_tip.getOrigin().z());
		frame_tip.M = KDL::Rotation::Quaternion(tf_tip.getRotation().x(), tf_tip.getRotation().y(), tf_tip.getRotation().z(), tf_tip.getRotation().w());
		
		frame_base.p = KDL::Vector(tf_base.getOrigin().x(), tf_base.getOrigin().y(), tf_base.getOrigin().z());
		frame_base.M = KDL::Rotation::Quaternion(tf_base.getRotation().x(), tf_base.getRotation().y(), tf_base.getRotation().z(), tf_base.getRotation().w());
		
		frame3.p = KDL::Vector(tf_d.getOrigin().x(), tf_d.getOrigin().y(), tf_d.getOrigin().z());
		frame3.M = KDL::Rotation::Quaternion(tf_d.getRotation().x(), tf_d.getRotation().y(), tf_d.getRotation().z(), tf_d.getRotation().w());
		
		
		if(!reset_markers_){
			point_ee.x = tf_tip.getOrigin().x();
			point_ee.y = tf_tip.getOrigin().y();
			point_ee.z = tf_tip.getOrigin().z();
			
			point_base.x = tf_base.getOrigin().x();
			point_base.y = tf_base.getOrigin().y();
			point_base.z = tf_base.getOrigin().z();
			
			point_ee_vec_.push_back(point_ee);
			point_base_vec_.push_back(point_base);
					
			double id1 = 0;
			double id2 = 1;
			
			showMarker(id1,1,0,0,"m",vis_pub_ee_,point_ee_vec_);
			showMarker(id2,0,1,0,"m",vis_pub_base_,point_base_vec_);
		}
		///----------------------------------------------------------------------
	
	
	if(base_active_)
	{	
		if(reset_markers_){
			point_ee_vec_.clear();
			point_base_vec_.clear();
		}
		
		q_dot_ik.resize(chain_.getNrOfJoints()+3);
		try{
			tf_listener_.waitForTransform("base_link",chain_tip_, ros::Time(0), ros::Duration(0.5));
			tf_listener_.lookupTransform("base_link",chain_tip_,  ros::Time(0), transform_tip_basefootprint);
		}catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			return;
		}
		frame.p = KDL::Vector(transform_tip_basefootprint.getOrigin().x(), transform_tip_basefootprint.getOrigin().y(), transform_tip_basefootprint.getOrigin().z());
		frame.M = KDL::Rotation::Quaternion(transform_tip_basefootprint.getRotation().x(), transform_tip_basefootprint.getRotation().y(), transform_tip_basefootprint.getRotation().z(), transform_tip_basefootprint.getRotation().w());
		
		
		try{
			tf_listener_.waitForTransform(chain_base_,"base_link", ros::Time(0), ros::Duration(0.5));
			tf_listener_.lookupTransform(chain_base_,"base_link", ros::Time(0), transform_base_basefootprint);
		}catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			return;
		}
		frame2.p = KDL::Vector(transform_base_basefootprint.getOrigin().x(), transform_base_basefootprint.getOrigin().y(), transform_base_basefootprint.getOrigin().z());
		frame2.M = KDL::Rotation::Quaternion(transform_base_basefootprint.getRotation().x(), transform_base_basefootprint.getRotation().y(), transform_base_basefootprint.getRotation().z(), transform_base_basefootprint.getRotation().w());
		
		ret_ik = p_augmented_solver_->CartToJnt(last_q_, last_q_dot_, twist, q_dot_ik,&limits_min_,&limits_max_,frame,frame2);
		
	}
	
	if(base_compensation_)
	{
		twist = getBaseCompensatedTwist(twist,twist_odometry_);
	
		/// DEBUG
		//KDL::Frame frame3;
		//tf::StampedTransform tf_debug;
		//try{
			//tf_listener_.waitForTransform("base_link",chain_base_, ros::Time(0), ros::Duration(0.5));
			//tf_listener_.lookupTransform("base_link",chain_base_, ros::Time(0), tf_debug);
		//}catch (tf::TransformException ex){
			//ROS_ERROR("%s",ex.what());
			//return;
		//}
		
		//try{
			//tf_listener_.waitForTransform("odom_combined",chain_tip_, ros::Time(0), ros::Duration(0.5));
			//tf_listener_.lookupTransform("odom_combined",chain_tip_, ros::Time(0), tf_tip);
		//}catch (tf::TransformException ex){
			//ROS_ERROR("%s",ex.what());
			//return;
		//}
		//try{
			//tf_listener_.waitForTransform("odom_combined","base_link", ros::Time(0), ros::Duration(0.5));
			//tf_listener_.lookupTransform("odom_combined","base_link", ros::Time(0), tf_base);
		//}catch (tf::TransformException ex){
			//ROS_ERROR("%s",ex.what());
			//return;
		//}
		//frame_tip.p = KDL::Vector(tf_tip.getOrigin().x(), tf_tip.getOrigin().y(), tf_tip.getOrigin().z());
		//frame_tip.M = KDL::Rotation::Quaternion(tf_tip.getRotation().x(), tf_tip.getRotation().y(), tf_tip.getRotation().z(), tf_tip.getRotation().w());
		//
		//frame_base.p = KDL::Vector(tf_base.getOrigin().x(), tf_base.getOrigin().y(), tf_base.getOrigin().z());
		//frame_base.M = KDL::Rotation::Quaternion(tf_base.getRotation().x(), tf_base.getRotation().y(), tf_base.getRotation().z(), tf_base.getRotation().w());
		//
		//
		//frame3.p = KDL::Vector(tf_debug.getOrigin().x(), tf_debug.getOrigin().y(), tf_debug.getOrigin().z());
		//frame3.M = KDL::Rotation::Quaternion(tf_debug.getRotation().x(), tf_debug.getRotation().y(), tf_debug.getRotation().z(), tf_debug.getRotation().w());
		//
		//tf::PoseKDLToMsg(frame_tip,pose_tip);
		//tf::PoseKDLToMsg(frame_base,pose_base);
		
		
		//geometry_msgs::Twist twist_manipulator_in_base_link;
		
		//Twist Manipulator in base_link
		//tf::twistKDLToMsg(frame3*twist,twist_manipulator_in_base_link);
		
		//pose_base_pub_.publish(pose_base);
		//pose_tip_pub_.publish(pose_tip);
		
		//twist_real_pub_.publish(twist_manipulator_in_base_link);	// Base_link
		///----------------------------------------------------------------------
	}
	
	
	if(!base_active_){
		ret_ik = p_augmented_solver_->CartToJnt(last_q_, last_q_dot_, twist, q_dot_ik,&limits_min_,&limits_max_);
	}
	
	if(ret_ik < 0)
	{
		ROS_ERROR("No Vel-IK found!");
	}
	else
	{
		if(base_active_){
			///normalize guarantees that velocities are within limits --- only needed for CartToJnt without damping
			q_dot_ik = normalize_velocities(q_dot_ik);
		}
		
		std_msgs::Float64MultiArray vel_msg;
		for(unsigned int i=0; i<dof_; i++)
		{
			vel_msg.data.push_back(q_dot_ik(i));
			//ROS_WARN("DesiredVel %d: %f", i, q_dot_ik(i));
		}
		if(base_active_)
		{
			geometry_msgs::Twist base_vel_msg;
			/// Base Velocities with respect to base_link
			base_vel_msg.linear.x = q_dot_ik(dof_);
			base_vel_msg.linear.y = q_dot_ik(dof_+1);
			base_vel_msg.angular.z = q_dot_ik(dof_+2);
			
			//if(std::fabs(q_dot_ik(dof_)) > params_.base_min_vel){
				//base_vel_msg.linear.x = q_dot_ik(dof_);
			//}else{
				//base_vel_msg.linear.x = 0;
			//}
			//
			//if(std::fabs(q_dot_ik(dof_+1)) > params_.base_min_vel){
				//base_vel_msg.linear.y = q_dot_ik(dof_+1);
			//}else{
				//base_vel_msg.linear.y = 0;
			//}
			//
			//if(std::fabs(q_dot_ik(dof_+2)) > params_.base_min_vel){
				//base_vel_msg.angular.z = q_dot_ik(dof_+2);
			//}else{
				//base_vel_msg.angular.z = 0;
			//}
			base_vel_pub.publish(base_vel_msg);	// base_link
			
			
			KDL::Twist twist_mani,twist_base;
			geometry_msgs::Twist twist_mani_msg,twist_base_msg,twist_combined_msg;
			
			tf::twistMsgToKDL(base_vel_msg, twist_base);
			//twist_base = frame_base * twist_base; // odom_combined
			tf::twistKDLToMsg(twist_base,twist_base_msg); 
			twist_real_base_pub_.publish(twist_base_msg);// arm_left/debug/twist_base/linear
			
			KDL::Twist twist2;
			KDL::FrameVel FrameVel;
			KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(last_q_,last_q_dot_);

			jntToCartSolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);
			int ret = jntToCartSolver_vel_->JntToCart(jntArrayVel,FrameVel,-1);

			if(ret>=0){
				twist2 = FrameVel.GetTwist();
				twist2 = frame3 * twist2;	// base_link
				tf::twistKDLToMsg(twist2,twist_mani_msg);	// Manipulator twist
			}
			else{
				ROS_WARN("ChainFkSolverVel failed!");
			}	
				
				//twist_mani= frame3 * twist;// - twist_base; // odom_combined
				//tf::twistKDLToMsg(twist_mani,twist_mani_msg);
				twist_real_pub_.publish(twist_mani_msg); // arm_left/debug/twist_mani/linear
				
				tf::twistKDLToMsg(twist_base+twist2,twist_combined_msg);
				soll_twist_pub_.publish(twist_combined_msg);
			}
		vel_pub.publish(vel_msg);
		
		/////---------------------------------------------------------------------
		/// Normalized q_dot into Twist
		//KDL::FrameVel FrameVel;
		//geometry_msgs::Twist twist_msg;
		//KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(last_q_,q_dot_ik);
		
		//int ret = p_fksolver_vel_->JntToCart(jntArrayVel,FrameVel,-1);
		
		//if(ret>=0)
		//{
			//KDL::Twist twist = FrameVel.GetTwist();
			//tf::twistKDLToMsg(twist,twist_msg);
		//}
		//twist_real_base_pub_.publish(twist_msg);
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

void CobTwistController::odometry_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	tf::StampedTransform transform_tf,transform_footprint_tip,transform_chain,transform_base_tip;
	KDL::Frame frame,frame_ft,frame_combined;
	KDL::Twist twist_odometry, twist_odometry_transformed, tangential_twist;
	KDL::Twist angular_twist;
	geometry_msgs::Twist twister;
	double roll,pitch,yaw;
	
	try{
		tf_listener_.waitForTransform(chain_base_,"base_link", ros::Time(0), ros::Duration(0.5));
		tf_listener_.lookupTransform(chain_base_,"base_link", ros::Time(0), transform_tf);
		
		tf_listener_.waitForTransform("base_link",chain_tip_, ros::Time(0), ros::Duration(0.5));
		tf_listener_.lookupTransform("base_link",chain_tip_, ros::Time(0), transform_footprint_tip);
		
		frame.p = KDL::Vector(transform_tf.getOrigin().x(), transform_tf.getOrigin().y(), transform_tf.getOrigin().z());
		frame.M = KDL::Rotation::Quaternion(transform_tf.getRotation().x(), transform_tf.getRotation().y(), transform_tf.getRotation().z(), transform_tf.getRotation().w());
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}	
	// Calculate tangential twist for angular base movements v = w x r
	Eigen::Vector3d r(transform_footprint_tip.getOrigin().x(),transform_footprint_tip.getOrigin().y(),transform_footprint_tip.getOrigin().z());
	Eigen::Vector3d w(0,0,msg->twist.twist.angular.z);
	Eigen::Vector3d res = w.cross(r);
	tangential_twist.vel = KDL::Vector(res(0),res(1),res(2));
	tangential_twist.rot = KDL::Vector(0,0,0);
	//ROS_INFO("Crossproduct : %f %f %f",res(0),res(1),res(2));
	//ROS_INFO("TCP: %f %f %f",transform_footprint_tip.getOrigin().x(),transform_footprint_tip.getOrigin().y(),transform_footprint_tip.getOrigin().z());
	
	tf::twistMsgToKDL(msg->twist.twist, twist_odometry);	// Base Twist
	twist_odometry_raw_ = twist_odometry;
	
	// transform into chain_base
	twist_odometry_transformed = frame * (twist_odometry+tangential_twist);
	
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
	
	///Do we need this? maybe
	if(base_active_)
	{
		//TEST: limit base_velocities
		double max_trans_velocity = 1;
		double max_rot_velocity = 0.5;
		if(max_factor < std::fabs((q_dot_ik(dof_)/max_trans_velocity)))
		{
			max_factor = std::fabs((q_dot_ik(dof_)/max_trans_velocity));
			//ROS_WARN("BaseTransX exceeds limit: Desired %f, Limit %f, Factor %f", q_dot_ik(dof_), max_trans_velocity, max_factor);
		}
		if(max_factor < std::fabs((q_dot_ik(dof_+1)/max_trans_velocity)))
		{
			max_factor = std::fabs((q_dot_ik(dof_+1)/max_trans_velocity));
			//ROS_WARN("BaseTransY exceeds limit: Desired %f, Limit %f, Factor %f", q_dot_ik(dof_+1), max_trans_velocity, max_factor);
		}
		if(max_factor < std::fabs((q_dot_ik(dof_+2)/max_rot_velocity)))
		{
			max_factor = std::fabs((q_dot_ik(dof_+2)/max_rot_velocity));
			//ROS_WARN("BaseRotZ exceeds limit: Desired %f, Limit %f, Factor %f", q_dot_ik(dof_+2), max_rot_velocity, max_factor);
		}
	}
	
	if(max_factor > 1)
	{
		//ROS_INFO("Normalizing velocities!");
		for(unsigned int i=0; i<dof_; i++)
		{
			q_dot_norm(i) = q_dot_ik(i)/max_factor;
			//ROS_WARN("Joint %d Normalized %f", i, q_dot_norm(i));
		}
		
		if(base_active_){
			q_dot_norm(dof_) = q_dot_ik(dof_)/max_factor;
			q_dot_norm(dof_+1) = q_dot_ik(dof_+1)/max_factor;
			q_dot_norm(dof_+2) = q_dot_ik(dof_+2)/max_factor;
		}
	}
	
	return q_dot_norm;
}



std::vector<double> CobTwistController::normalize_velocities_test(KDL::JntArray q_dot_ik, double base_x, double base_y)
{
	KDL::JntArray q_dot_norm = q_dot_ik;
	std::vector<double> ret;
	double max_factor = 1;
	double vel_max = 0.5;
	
	
	for(unsigned int i=0; i<dof_; i++)
	{
		if(max_factor < std::fabs((q_dot_ik(i)/limits_vel_[i])))
		{
			max_factor = std::fabs((q_dot_ik(i)/limits_vel_[i]));
			ROS_WARN("Joint %d exceeds limit: Desired %f, Limit %f, Factor %f", i, q_dot_ik(i), limits_vel_[i], max_factor);
		}
	}
	//if(max_factor < std::fabs(base_x/vel_max)){
		//max_factor = std::fabs(base_x/vel_max);
	//}
	//
	//if(max_factor < std::fabs(base_y/vel_max)){
		//max_factor = std::fabs(base_y/vel_max);
	//}
	
	if(max_factor > 1)
	{
		ROS_INFO("Normalizing velocities!");
		for(unsigned int i=0; i<dof_; i++)
		{
			ret.push_back(q_dot_ik(i)/max_factor);
		}
		
		//ret.push_back(base_x/max_factor);
		//ret.push_back(base_y/max_factor);
		//ret.push_back(q_dot_ik(dof_)/max_factor);
	}
	return ret;
}

void CobTwistController::showMarker(int marker_id,double red, double green, double blue, std::string ns, ros::Publisher pub, std::vector<geometry_msgs::Point> &pos_v)
{
	
  	visualization_msgs::Marker marker;
	marker.header.frame_id = "odom_combined";
	marker.header.stamp = ros::Time();
	marker.ns = ns;
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.01;
	
	marker.color.r = red;
	marker.color.g = green;
	marker.color.b = blue;
	
	marker.points.insert(marker.points.begin(), pos_v.begin(), pos_v.end()); 
	
	marker.color.a = 1.0;
	pub.publish( marker );
}
