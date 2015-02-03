/*
 * cob_frame_tracker_action_client.cpp
 *
 *  Created on: Jan 14, 2015
 *      Author: ipa-fmw-ce
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_frame_tracker/FrameTrackingAction.h>


#include <cob_frame_tracker_action_client/cob_frame_tracker_action_client.h>


bool CobFrameTrackerActionClient::initialize()
{
	ros::NodeHandle nh_client("cob_frame_tracker_action_client_node");
	start_server_ = nh_client.advertiseService("start_tracking", &CobFrameTrackerActionClient::start_tracking_cb, this);
	stop_server_ = nh_client.advertiseService("stop_tracking", &CobFrameTrackerActionClient::stop_tracking_cb, this);

	tracking_frame_ = active_frame_;
	tracking_ = false;

	ROS_INFO("Waiting for action server to start.");
	ac_.waitForServer();

	//define standard goal:

	goal_.stop_on_goal = false;
	goal_.tracking_frame = "active_frame";
	goal_.tracking_time = 0;

	ROS_INFO("Server started ... initialized!");
	return true;
}

void CobFrameTrackerActionClient::run()
{
	ros::Rate r(10);
	ROS_INFO("Client running ...");
	ros::spin();
}

bool CobFrameTrackerActionClient::start_tracking_cb(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response)
{
	tracking_frame_ = request.data;
	tracking_ = true;

	response.success = true;

	//new:
	ROS_INFO("Service started ... sending goal.");
	goal_.tracking_frame = tracking_frame_;
	// send a goal to the action
	ac_.sendGoal(goal_);
	ROS_INFO("printing goal state:");
	actionlib::SimpleClientGoalState state = ac_.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());
	return true;
}

bool CobFrameTrackerActionClient::stop_tracking_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	if (tracking_)
	{
		tracking_frame_ = active_frame_;
		tracking_ = false;

		ac_.cancelAllGoals();
		ROS_INFO("All goals canceled");

		ROS_INFO("printing goal state:");
		actionlib::SimpleClientGoalState state = ac_.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());

		return true;
	}
	else
	{
		ROS_INFO("Invalid call: There is no goal running. Stop tracking denied.");
		return false;
	}

}


