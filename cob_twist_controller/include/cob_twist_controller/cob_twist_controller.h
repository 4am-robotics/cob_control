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
 *   ROS stack name: cob_driver
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
#ifndef COB_TWIST_CONTROLLER_H
#define COB_TWIST_CONTROLLER_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <brics_actuator/JointVelocities.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <cob_twist_controller/augmented_solver.h>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>

#include <tf/transform_listener.h>


class CobTwistController
{
private:
	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_;
	
	ros::Subscriber jointstate_sub;
	ros::Subscriber twist_sub;
	ros::Publisher vel_pub;
	
	KDL::Chain chain_;
	std::string chain_base_;
	std::string chain_tip_;
	
	KDL::ChainIkSolverVel_pinv* p_iksolver_vel_;
	augmented_solver* p_augmented_solver_;
	
	std::vector<std::string> joints_;
	unsigned int dof_;
	std::vector<float> limits_min_;
	std::vector<float> limits_max_;
	std::vector<float> limits_vel_;
	
	KDL::JntArray last_q_;
	KDL::JntArray last_q_dot_;
	
	
public:
	CobTwistController() {;}
	~CobTwistController();
	
	void initialize();
	void run();
	
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);
	void twist_cb(const geometry_msgs::Twist::ConstPtr& msg);
	
	
	KDL::JntArray normalize_velocities(KDL::JntArray q_dot_ik);
};

#endif

