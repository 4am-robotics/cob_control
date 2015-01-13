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

#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <cob_multicomp_twist_controller/MultiCompTwistControllerConfig.h>

struct MultiCompTwistControllerParams {
    bool base_compensation;
    bool base_active;
    double base_ratio;
};

class CobMultiCompTwistController
{
private:
	ros::NodeHandle nh_;
	
	bool base_compensation_;
	bool base_active_;

	MultiCompTwistControllerParams params_;
	

public:
	CobMultiCompTwistController():
		base_compensation_(false),
		base_active_(false)
	{;}
	~CobMultiCompTwistController();
	
	bool initialize();
	void run();
	
	boost::recursive_mutex reconfig_mutex_;
	boost::shared_ptr<dynamic_reconfigure::Server<cob_multicomp_twist_controller::MultiCompTwistControllerConfig> > reconfigure_server_;
	void reconfigure_callback(cob_multicomp_twist_controller::MultiCompTwistControllerConfig &config, uint32_t level);

	void SetTwistControllerParamsParams(MultiCompTwistControllerParams params){params_ = params;}

};
#endif
