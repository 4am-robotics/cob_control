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
	CobFrameTracker(std::string name):
	as_(nh_, name, false),
	cart_min_dist_threshold_lin_(0.01),
	cart_min_dist_threshold_rot_(0.01),
	twist_dead_threshold_lin_(0.05),
	twist_dead_threshold_rot_(0.05),
	twist_deviation_threshold_lin_(0.5),
	twist_deviation_threshold_rot_(0.5)
	{
		as_.registerGoalCallback(boost::bind(&CobFrameTracker::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&CobFrameTracker::preemptCB, this));
		as_.start();
	}
	~CobFrameTracker();
	
	bool initialize();
	void run();
	void goalCB();
	void preemptCB();
	void succeed();
	void abort();
	bool searchForAbortionCriteria();
	
	void publish_twist(ros::Duration period);
	bool start_tracking_cb(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response);
	bool stop_tracking_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);
	bool checkDistance(const double dist, const double rot);
	bool checkDeviationErrors(const KDL::Twist current, const KDL::Twist target);
	bool checkNoTwistErrors(const KDL::Twist current);
	//reconfigure params
	boost::recursive_mutex reconfig_mutex_;
	boost::shared_ptr< dynamic_reconfigure::Server<cob_frame_tracker::FrameTrackerConfig> > reconfigure_server_;
	void reconfigure_callback(cob_frame_tracker::FrameTrackerConfig &config, uint32_t level);

	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_;

	std::vector<std::string> joints_;
	unsigned int dof_;
	std::string action_name_;
	actionlib::SimpleActionServer<cob_frame_tracker::FrameTrackingAction> as_;
	cob_frame_tracker::FrameTrackingFeedback feedback_;
	cob_frame_tracker::FrameTrackingResult result_;
	ros::Subscriber jointstate_sub;
	
	//goal definition:
	bool stop_on_goal_;
	int tracking_time_;
	ros::Time start_of_tracking_;

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

	double cart_min_dist_threshold_lin_;
	double cart_min_dist_threshold_rot_;
	double twist_dead_threshold_lin_;
	double twist_dead_threshold_rot_;
	double twist_deviation_threshold_lin_;
	double twist_deviation_threshold_rot_;

	ros::Time timer;

	/// Abortion Criteria
	double cart_distance_;
	double rot_distance_;

	/// Helping vars:
	std::string abortion_message_;


private:
	double update_rate_;
	
	bool tracking_;
	bool tracking_goal_;
	std::string tracking_frame_;	//goal frame
	std::string chain_tip_link_;		//twists with respect to this frame
	
	double max_vel_lin_;
	double max_vel_rot_;
	
	ros::ServiceServer start_server_;
	ros::ServiceServer stop_server_;
	ros::Publisher twist_pub_;
	
	/// Debug
	ros::Publisher dif_pub_;
	
	bool movable_trans_;
	bool movable_rot_;
	
	control_toolbox::Pid pid_controller_trans_x_;       /**< Internal PID controller. */
	control_toolbox::Pid pid_controller_trans_y_;
	control_toolbox::Pid pid_controller_trans_z_;
	
	control_toolbox::Pid pid_controller_rot_x_;         /**< Internal PID controller. */
	control_toolbox::Pid pid_controller_rot_y_;
	control_toolbox::Pid pid_controller_rot_z_;

	// ABORTION CRITERIA:
	KDL::Twist current_twist_;
	KDL::Twist target_twist_;

	bool value_failed_;
	int numberOfFailedValues_;

};

#endif

