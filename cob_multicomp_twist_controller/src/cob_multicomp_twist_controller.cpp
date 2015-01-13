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
 *   This package provides a generic Twist controller for the Care-O-bot
 *
 ****************************************************************/
#include <ros/ros.h>

#include <cob_multicomp_twist_controller/cob_multicomp_twist_controller.h>




bool CobMultiCompTwistController::initialize()
{
	ros::NodeHandle nh_cartesian("cartesian_controller");

	///Setting up dynamic_reconfigure server for the AugmentedSolverParams
	reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_multicomp_twist_controller::MultiCompTwistControllerConfig>(reconfig_mutex_, nh_cartesian));
	reconfigure_server_->setCallback(boost::bind(&CobMultiCompTwistController::reconfigure_callback,   this, _1, _2));
	
	
	ROS_INFO("...initialized!");
	return true;
}


void CobMultiCompTwistController::run()
{
	ROS_INFO("cob_twist_controller...spinning");
	ros::spin();
}

void CobMultiCompTwistController::reconfigure_callback(cob_multicomp_twist_controller::MultiCompTwistControllerConfig &config, uint32_t level)
{

	base_compensation_ = config.base_compensation;
	base_active_ = config.base_active;
}


