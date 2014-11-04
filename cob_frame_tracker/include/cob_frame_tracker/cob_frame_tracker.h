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

class CobFrameTracker
{
public:
	CobFrameTracker() {;}
	~CobFrameTracker();
	
	bool initialize();
	void run();
	
	void publish_twist(ros::Duration period);
	bool start_tracking_cb(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response);
	bool stop_tracking_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	
	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_;
	
private:
	double update_rate_;
	
	bool tracking_;
	std::string tracking_frame_;	//goal frame
	std::string active_frame_;		//twists with respect to this frame
	double max_vel_lin_;
	double max_vel_rot_;
	
	ros::ServiceServer start_server_;
	ros::ServiceServer stop_server_;
	ros::Publisher twist_pub_;
	
	bool movable_trans_;
	bool movable_rot_;
	
	control_toolbox::Pid pid_controller_trans_x_;       /**< Internal PID controller. */
	control_toolbox::Pid pid_controller_trans_y_;
	control_toolbox::Pid pid_controller_trans_z_;
	
	control_toolbox::Pid pid_controller_rot_;         /**< Internal PID controller. */
};

#endif

