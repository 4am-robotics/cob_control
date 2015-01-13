/*
 * cob_tracking_action_client.h

 *
 *  Created on: Dec 15, 2014
 *      Author: ipa-fmw-ce
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_tracking_action/TrackingAction.h>
#include <std_srvs/Empty.h>
#include <cob_srvs/SetString.h>
#include <geometry_msgs/Twist.h>

#ifndef COB_TRACKING_ACTION_CLIENT_H_
#define COB_TRACKING_ACTION_CLIENT_H_

class CobTrackingActionClient
{
	public:
		CobTrackingActionClient():
		ac_("cob_tracking_action_node")
		{;}
		~CobTrackingActionClient();

		bool initialize();
		void run();
		bool start_tracking_cb(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response);
		bool stop_tracking_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

		ros::NodeHandle nh_;


	private:
		bool tracking_;
		actionlib::SimpleActionClient<cob_tracking_action::TrackingAction> ac_;
		cob_tracking_action::TrackingGoal goal_;
		ros::ServiceServer start_server_;
		ros::ServiceServer stop_server_;
		std::string tracking_frame_;	//goal frame
		std::string active_frame_;




};

#endif /* COB_TRACKING_ACTION_CLIENT_H_ */
