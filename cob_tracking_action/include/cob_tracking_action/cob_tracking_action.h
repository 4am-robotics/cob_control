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
#ifndef COB_TRACKING_ACTION_H
#define COB_TRACKING_ACTION_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
//#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include <algorithm>

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <cob_tracking_action/TrackingActionConfig.h>
#include <cob_tracking_action/TrackingAction.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <control_toolbox/pid.h>

struct TrackingActionParams {
    int timer_value;
    bool base_active;
    double base_ratio;
    double distance_threshold;
};


class TrackingAction
{
	public:

	TrackingAction(std::string name) :
		as_(nh_, name, false),
		action_name_(name),
		timer_value_(10),
		base_active_(false),
		distance_threshold_(0.005)
	{
		as_.registerGoalCallback(boost::bind(&TrackingAction::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&TrackingAction::preemptCB, this));
		as_.start();

	}

	~TrackingAction(void)
	{

	}

	bool initialize();
	void run();
	void goalCB();
	void preemptCB();
	void succeed();
	void abort();
	bool searchForAbortionCriteria();
//	void readXdotCurrent();
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);
	void updateList(KDL::Twist twist,  double time_filter_threshold);
	void updateList2(double norm, double time_filter_threshold);
	void calcMean();
	double meanNorm();

	void publish_twist(ros::Duration period);
	//reconfigure params
	boost::recursive_mutex reconfig_mutex_;
	boost::shared_ptr< dynamic_reconfigure::Server<cob_tracking_action::TrackingActionConfig> > reconfigure_server_;
	void reconfigure_callback(cob_tracking_action::TrackingActionConfig &config, uint32_t level);
	void SetTrackingActionParams(TrackingActionParams params){params_ = params;}

	tf::TransformListener tf_listener_;

protected:

	ros::NodeHandle nh_;
	std::vector<std::string> joints_;
	unsigned int dof_;
	std::string action_name_;
	actionlib::SimpleActionServer<cob_tracking_action::TrackingAction> as_;
	cob_tracking_action::TrackingFeedback feedback_;
	cob_tracking_action::TrackingResult result_;
//	cob_tracking_action::TrackingGoal goal_;
	ros::Subscriber jointstate_sub;

	//goal definition:
	std::string target_tracking_frame_;
	bool reachable_goal_;
	int tracking_time_;
	ros::Time start_of_tracking_;

	//FrameTrackerInsert:
	double update_rate_;
	ros::Publisher twist_pub_;
	ros::Publisher debug_pub_1;
	ros::Publisher debug_pub_2;
	ros::Publisher debug_pub_3;
	ros::Publisher debug_pub_4;

	bool tracking_;
	//std::string tracking_frame_;	//goal frame
	std::string active_frame_;		//twists with respect to this frame
	double max_vel_lin_;
	double max_vel_rot_;

	bool movable_trans_;
	bool movable_rot_;

	control_toolbox::Pid pid_controller_trans_x_;       /**< Internal PID controller. */
	control_toolbox::Pid pid_controller_trans_y_;
	control_toolbox::Pid pid_controller_trans_z_;

	control_toolbox::Pid pid_controller_rot_;
	//EndOfFrameTrackerInsert:

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
	bool base_active_;
	double base_ratio_;

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


};
#endif
