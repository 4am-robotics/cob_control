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

bool CobTwistControllerAction::initialize()
{
	ros::NodeHandle nh_action("twist_controller_action");
	
	// JointNames
	//if(!nh_.getParam("joint_names", joints_))
	//{
	//	ROS_ERROR("Parameter 'joint_names' not set");
	//	return false;
	//}
	//dof_ = joints_.size();
	//
	ROS_INFO("...initialized!");
	return true;
}


void CobTwistControllerAction::run()
{
	ROS_INFO("cob_twist_controller_action...spinning");
	ros::spin();
}
