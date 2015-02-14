/*
 * cob_frame_tracker_action_client.h
 *
 *  Created on: Jan 14, 2015
 *      Author: ipa-fmw-ce
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cob_frame_tracker/FrameTrackingAction.h>
#include <std_srvs/Empty.h>
#include <cob_srvs/SetString.h>
#include <geometry_msgs/Twist.h>

#ifndef COB_FRAME_TRACKER_ACTION_CLIENT_H_
#define COB_FRAME_TRACKER_ACTION_CLIENT_H_

class CobFrameTrackerActionClient
{
	public:
		CobFrameTrackerActionClient():
			ac_("frame_tracker")
		{;}
		~CobFrameTrackerActionClient();

		bool initialize();
		void run();
		bool start_tracking_cb(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response);
		bool stop_tracking_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

		ros::NodeHandle nh_;

	private:
		bool tracking_;
		actionlib::SimpleActionClient<cob_frame_tracker::FrameTrackingAction> ac_;
		cob_frame_tracker::FrameTrackingGoal goal_;
		ros::ServiceServer start_server_;
		ros::ServiceServer stop_server_;
		std::string tracking_frame_;	//goal frame
		std::string active_frame_;
};



#endif /* COB_FRAME_TRACKER_ACTION_CLIENT_H_ */
