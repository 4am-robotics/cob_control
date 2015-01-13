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

#include <ros/ros.h>
#include <cob_tracking_action_client/cob_tracking_action_client.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_tracking_action_client");
	CobTrackingActionClient *ctac = new CobTrackingActionClient();

	if(ctac->initialize())
	{
		ctac->run();
	}
	else
		ROS_ERROR("Failed to initialize TrackingActionClient");

	return 0;
}


