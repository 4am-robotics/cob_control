#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_tracking_action/TrackingAction.h>
//#include <boost/thread.hpp>
//
//void spinThread()
//{
//  ros::spin();
//}

#include <cob_tracking_action_client/cob_tracking_action_client.h>




bool CobTrackingActionClient::initialize()
{
	ros::NodeHandle nh_client("client_node");
	start_server_ = nh_.advertiseService("start_tracking", &CobTrackingActionClient::start_tracking_cb, this);
	stop_server_ = nh_.advertiseService("stop_tracking", &CobTrackingActionClient::stop_tracking_cb, this);

	// create the action client
	//ac_("cob_tracking_action_client_node");

	//ac_.SimpleActionClient("cob_tracking_action_client_node");

	//boost::thread spin_thread(&spinThread);

	tracking_frame_ = active_frame_;
	tracking_ = false;

	ROS_INFO("Waiting for action server to start.");
	ac_.waitForServer();

	//define standard goal:

	goal_.reachable_goal = false;
	goal_.tracking_frame = "active_frame";
	goal_.tracking_time = 20;

	ROS_INFO("Server started ... initialized!");
	return true;
}

void CobTrackingActionClient::run()
{
	ros::Rate r(10);
	ROS_INFO("Client running ...");
	ros::spin();
}

bool CobTrackingActionClient::start_tracking_cb(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response)
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

bool CobTrackingActionClient::stop_tracking_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
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






