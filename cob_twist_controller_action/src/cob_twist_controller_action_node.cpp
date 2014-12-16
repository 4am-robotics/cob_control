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
 *   ROS package name: cob_twist_controller_action
 *
 * \author
 *   Author: Christian Ehrmann, email: christian.ehrmann@ipa.fraunhofer.de
 *
 * \date Date of creation: November, 2014
 *
 * \brief
 *   ...
 *
 ****************************************************************/
#include <ros/ros.h>
#include <cob_twist_controller_action/cob_twist_controller_action.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "cob_twist_controller_action_node");
	CobTwistControllerAction *cob_twist_controller_action = new CobTwistControllerAction();
	
	if(cob_twist_controller_action->initialize())
	{
		cob_twist_controller_action->run();
	}
	else
		ROS_ERROR("Failed to initialize ControllerAction");
	
	return 0;
}
