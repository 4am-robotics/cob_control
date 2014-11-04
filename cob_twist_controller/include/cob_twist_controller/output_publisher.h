
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
 *   ROS package name: cob_articulation
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de
 *
 * \date Date of creation: September, 2014
 *
 * \brief
 *   ...
 *
 ****************************************************************/
#ifndef output_publisher_H
#define output_publisher_H

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/framevel.hpp>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <cob_srvs/SetString.h>


class OutputPublisher
{
public:
	void initialize();
	void run();
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);						 
	geometry_msgs::Pose getEndeffectorPose();
	geometry_msgs::Pose getTrackingFramePosition();
	
private:
	ros::NodeHandle nh_;
	ros::Subscriber jointstate_sub_;
	ros::Publisher end_eff_vel_pub_;
	ros::Publisher end_eff_pos_pub_;
	ros::Publisher manipulability_pub_;
	ros::Publisher rcond_pub_;
	void euler(std::vector<double> *out, double in, double dt);	
	
	
	/// KDL Conversion
	KDL::Chain chain_;
	std::string chain_base_;
	std::string chain_tip_;
	KDL::JntArray last_q_;
	KDL::JntArray last_q_dot_;
	std::vector<std::string> joints_;		
	KDL::JntArrayVel JntArrayVel_;
	KDL::ChainFkSolverVel_recursive* jntToCartSolver_vel_;
	KDL::ChainFkSolverPos_recursive* jntToCartSolver_pos_;	
	unsigned int dof_;
	KDL::Vector vector_vel_,vector_rot_,vector_pos_lin_, vector_pos_ang_;
	
	double q_x_lin_in	,q_y_lin_in		,q_z_lin_in;
	double q_x_lin_out	,q_y_lin_out	,q_z_lin_out;
	
	/// Transform Listener
   	tf::Transform transform_;
   	tf::Quaternion q_;
   	tf::TransformListener listener_;
   	tf::TransformBroadcaster br_;
   	tf::StampedTransform stampedTransform_;
	double roll_,pitch_,yaw_;
	
	std::vector <double> timeVect_;
	std::string referenceFrame_,endeffectorFrame_,trackingFrame_;
	ros::Timer timer_stop_;
	
	/// Euler Integration
	double dt_;
	
	double wkm1;
	bool initial_iteration;
};

#endif

