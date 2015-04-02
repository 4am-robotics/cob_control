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
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <cob_twist_controller/augmented_solver.h>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <cob_twist_controller/TwistControllerConfig.h>
#include <Eigen/Dense>


class CobTwistController
{
private:
	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_;
	
	ros::Time last_update_time_,time_;
	ros::Duration period_;
	
	ros::Subscriber jointstate_sub;
	ros::Subscriber odometry_sub;
	ros::Subscriber twist_sub;
	ros::Subscriber twist_stamped_sub;
	ros::Subscriber base_sub;
	ros::Publisher vel_pub;
	ros::Publisher base_vel_pub;
	ros::Publisher twist_pub_;
	ros::Publisher twist_current_pub_;
	
	KDL::Chain chain_;
	std::string chain_base_link_;
	std::string chain_tip_link_;
	
	KDL::ChainFkSolverVel_recursive* p_fksolver_vel_;
	KDL::ChainIkSolverVel_pinv* p_iksolver_vel_;
	AugmentedSolver* p_augmented_solver_;
	
	std::vector<std::string> joints_;
	unsigned int dof_;
	std::vector<double> limits_min_;
	std::vector<double> limits_max_;
	std::vector<double> limits_vel_;
	
	double max_vel_lin_;
	double max_vel_rot_;
	double max_vel_lin_base_;
	double max_vel_rot_base_;
	
	KDL::JntArray last_q_;
	KDL::JntArray last_q_dot_;
	
	bool enforce_limits_;
	bool base_compensation_;
	bool base_active_;
	bool reset_markers_;
	
	KDL::Twist twist_odometry_cb_;
	
	///Debug
	ros::Publisher 	debug_base_compensation_visual_tip_pub_,debug_base_compensation_visual_base_pub_,debug_base_compensation_pose_base_pub_,
					debug_base_compensation_pose_tip_pub_,debug_base_compensation_twist_manipulator_pub_,debug_base_active_twist_manipulator_pub_,
					debug_base_active_twist_base_pub_,debug_base_active_twist_ee_pub_;
	std::vector<geometry_msgs::Point> point_base_vec_,point_ee_vec_;
	KDL::ChainFkSolverVel_recursive* jntToCartSolver_vel_;
	
	////Debug
	tf::StampedTransform 	odom_transform_ct,
							odom_transform_bl,
							bl_transform_cb,
							bl_transform_ct,
							cb_transform_bl;
	
	KDL::Frame 	odom_frame_ct,
				odom_frame_bl,
				bl_frame_cb,
				bl_frame_ct,
				cb_frame_bl;
	
	void initAugmentedSolverParams();

	
public:
	CobTwistController():
		base_compensation_(false),
		base_active_(false),
		reset_markers_(false)
	{;}
	~CobTwistController();
	
	bool initialize();
	void run();
	
	boost::recursive_mutex reconfig_mutex_;
	boost::shared_ptr< dynamic_reconfigure::Server<cob_twist_controller::TwistControllerConfig> > reconfigure_server_;
	void reconfigure_callback(cob_twist_controller::TwistControllerConfig &config, uint32_t level);
	
	
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);
	void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg);
	void twist_cb(const geometry_msgs::Twist::ConstPtr& msg);
	void base_twist_cb(const geometry_msgs::Twist::ConstPtr& msg);
	
	void twist_stamped_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void solve_twist(KDL::Twist twist);
	
	KDL::JntArray normalize_velocities(KDL::JntArray q_dot_ik);
	
	///Debug
	void showMarker(int marker_id,double red, double green, double blue, std::string ns, ros::Publisher pub, std::vector<geometry_msgs::Point> &pos_v);
	void debug();

};
#endif
