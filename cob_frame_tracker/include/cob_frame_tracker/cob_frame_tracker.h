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
#ifndef COB_FRAME_TRACKER_H
#define COB_FRAME_TRACKER_H

#include <math.h>
#include <algorithm>
#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <cob_srvs/SetString.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <cob_frame_tracker/FrameTrackerConfig.h>
#include <cob_frame_tracker/FrameTrackingAction.h>

#include <control_toolbox/pid.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>

#include <control_toolbox/pid.h>


class CobFrameTracker
{
public:
	CobFrameTracker() {;}
	~CobFrameTracker() {;}
	
	bool initialize();
	void run(const ros::TimerEvent& event);
	
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);
	void publish_twist(ros::Duration period);
	bool start_tracking_cb(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response);
	bool stop_tracking_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	
	/// Action interface
	void goalCB();
	void preemptCB();
	void action_success();
	void action_abort();

private:
	double update_rate_;
	ros::Timer timer_;
	
	bool tracking_;
	bool tracking_goal_;
	std::string tracking_frame_;
	std::string chain_tip_link_;
	
	double max_vel_lin_;
	double max_vel_rot_;
	
	std::vector<std::string> joints_;
	unsigned int dof_;
	
	bool movable_trans_;
	bool movable_rot_;
	
	control_toolbox::Pid pid_controller_trans_x_;       /**< Internal PID controller. */
	control_toolbox::Pid pid_controller_trans_y_;
	control_toolbox::Pid pid_controller_trans_z_;
	
	control_toolbox::Pid pid_controller_rot_x_;         /**< Internal PID controller. */
	control_toolbox::Pid pid_controller_rot_y_;
	control_toolbox::Pid pid_controller_rot_z_;
	
	/// KDL Conversion
	KDL::Chain chain_;
	KDL::JntArray q_temp;
	KDL::JntArray q_dot_temp;
	KDL::JntArray last_q_;
	KDL::JntArray last_q_dot_;
	KDL::ChainFkSolverVel_recursive* jntToCartSolver_vel_;
	KDL::Vector vector_vel_,vector_rot_;
	std::string chain_base_;
	std::string chain_tip_;
	KDL::Tree my_tree;
	std::string robot_desc_string;
	
	/// ROS interface
	tf::TransformListener tf_listener_;
	
	ros::Subscriber jointstate_sub;
	ros::Publisher twist_pub_;

	ros::Publisher error_pub_;

	ros::ServiceServer start_server_;
	ros::ServiceServer stop_server_;
	
	/// Action interface
	std::string action_name_;
	actionlib::SimpleActionServer<cob_frame_tracker::FrameTrackingAction> *as_;
	cob_frame_tracker::FrameTrackingFeedback action_feedback_;
	cob_frame_tracker::FrameTrackingResult action_result_;
	
	boost::recursive_mutex reconfig_mutex_;
	boost::shared_ptr< dynamic_reconfigure::Server<cob_frame_tracker::FrameTrackerConfig> > reconfigure_server_;
	void reconfigure_callback(cob_frame_tracker::FrameTrackerConfig &config, uint32_t level);
	
	/// ABORTION CRITERIA:
	int checkStatus();
	bool checkInfinitesimalTwist(const KDL::Twist current);
	bool checkCartDistanceViolation(const double dist, const double rot);
	bool checkTwistViolation(const KDL::Twist current, const KDL::Twist target);
	
	bool stop_on_goal_;
	double tracking_duration_;
	ros::Time tracking_start_time_;
	
	double cart_min_dist_threshold_lin_;
	double cart_min_dist_threshold_rot_;
	double twist_dead_threshold_lin_;
	double twist_dead_threshold_rot_;
	double twist_deviation_threshold_lin_;
	double twist_deviation_threshold_rot_;
	
	KDL::Twist current_twist_;
	KDL::Twist target_twist_;

	double cart_distance_;
	double rot_distance_;

	unsigned int abortion_counter_;
	unsigned int max_abortions_;
};

#endif

