/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
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
 *   ROS package name: cob_multicomp_twist_controller
 *
 * \author
 *   Author: Christian Ehrmann, email: Christian.Ehrmann@ipa.fraunhofer.de
 *
 * \date Date of creation: January, 2015
 *
 * \brief
 *   This package provides a generic multi component twist controller for the Care-O-bot
 *
 ****************************************************************/
#ifndef COB_MULTICOMP_TWIST_CONTROLLER_H
#define COB_MULTICOMP_TWIST_CONTROLLER_H

#include <ros/ros.h>


class CobMultiCompTwistController
{
private:
	ros::NodeHandle nh_;
	
	
public:
	CobMultiCompTwistController(){;}
	~CobMultiCompTwistController();
	
	bool initialize();
	void run();
	
};
#endif
