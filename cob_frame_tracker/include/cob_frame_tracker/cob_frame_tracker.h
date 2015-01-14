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
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <control_toolbox/pid.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

struct CobFrameTrackerParams {
	int timer_value;
	double distance_threshold;
};


class CobFrameTracker
{
public:
	CobFrameTracker(std::string name):
	as_(nh_, name, false),
	timer_value_(10),
	distance_threshold_(0.005)
	{
		as_.registerGoalCallback(boost::bind(&FrameTrackingAction::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&FrameTrackingAction::preemptCB, this));
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
	void updateList(KDL::Twist twist,  double time_filter_threshold);
	void updateList2(double norm, double time_filter_threshold);
	void calcMean();
	double meanNorm();

	void publish_twist(ros::Duration period);
	//reconfigure params
	boost::recursive_mutex reconfig_mutex_;
	boost::shared_ptr< dynamic_reconfigure::Server<cob_frame_tracker::FrameTrackingActionConfig> > reconfigure_server_;
	void reconfigure_callback(cob_frame_tracker::FrameTrackingActionConfig &config, uint32_t level);
	void SetFrameTrackingActionParams(FrameTrackingActionParams params){params_ = params;}
	tf::TransformListener tf_listener_;

protected:

	ros::NodeHandle nh_;
	std::vector<std::string> joints_;
	unsigned int dof_;
	std::string action_name_;
	actionlib::SimpleActionServer<cob_frame_tracker::FrameTrackingAction> as_;
	cob_tracking_action::TrackingFeedback feedback_;
	cob_tracking_action::TrackingResult result_;
	
	//goal definition:
	std::string target_tracking_frame_;
	bool reachable_goal_;
	int tracking_time_;
	ros::Time start_of_tracking_;

//my calcs begin:
	double eukl_dist_lin_;
	//my cals end

	//KDL:
	/// KDL Conversion
	KDL::Chain chain_;
	KDL::JntArray q_temp;
	KDL::JntArray q_dot_temp;
	KDL::JntArray last_q_;
	KDL::JntArray last_q_dot_;
//	std::vector<std::string> joints_;
	KDL::ChainFkSolverVel_recursive* jntToCartSolver_vel_;
//	KDL::ChainFkSolverVel_recursive* p_fksolver_vel_;
//	KDL::ChainIkSolverVel_pinv* p_iksolver_vel_;
//	unsigned int dof_;
	KDL::Vector vector_vel_,vector_rot_;
	std::string chain_base_;
	std::string chain_tip_;
	KDL::Tree my_tree;
	std::string robot_desc_string;

	TrackingActionParams params_;
	int timer_value_;
	double distance_threshold_;

	ros::Time timer;

	/// Abortion Criteria
	double cart_distance_;
	double rot_distance_;
	KDL::Twist current_twist_;
	KDL::Twist target_twist_;
	KDL::Twist diff_twist_;
	/// Helping vars:
	std::string abortion_message_;
	std::vector<std::vector<double> > list_of_twists_stamped_;
	std::vector<std::vector<double> > list_of_norms_stamped_;

private:
	double update_rate_;
	
	bool tracking_;
//	std::string tracking_frame_;	//goal frame
	std::string active_frame_;		//twists with respect to this frame
	std::string base_link_;
	
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
	
	control_toolbox::Pid pid_controller_rot_;         /**< Internal PID controller. */
};

#endif

