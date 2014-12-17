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
//#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>


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

class TrackingAction
{
	public:

	TrackingAction(std::string name) :
		as_(nh_, name, false),
		action_name_(name)
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

	void publish_twist(ros::Duration period);
	boost::recursive_mutex reconfig_mutex_;
	boost::shared_ptr< dynamic_reconfigure::Server<cob_tracking_action::TrackingActionConfig> > reconfigure_server_;
	void reconfigure_callback(cob_tracking_action::TrackingActionConfig &config, uint32_t level);
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

	//goal definition:
	std::string target_tracking_frame_;
	bool reachable_goal_;
	int tracking_time_;
	ros::Time start_of_tracking_;

	//FrameTrackerInsert:
	double update_rate_;
	ros::Publisher twist_pub_;

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

};
#endif
