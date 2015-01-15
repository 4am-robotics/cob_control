/*
 * cob_frame_tracker_action_client_node.cpp
 *
 *  Created on: Jan 14, 2015
 *      Author: ipa-fmw-ce
 */

#include <ros/ros.h>
#include <cob_frame_tracker_action_client/cob_frame_tracker_action_client.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_frame_tracker_action_client");
	CobFrameTrackerActionClient *cftac = new CobFrameTrackerActionClient();

	if(cftac->initialize())
	{
		cftac->run();
	}
	else
		ROS_ERROR("Failed to initialize CobFrameTrackerActionClient");

	return 0;
}


