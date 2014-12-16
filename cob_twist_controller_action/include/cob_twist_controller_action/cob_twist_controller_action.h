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
 *   ROS package name: cob_twist_action
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
#ifndef COB_TWIST_CONTROLLER_ACTION_H
#define COB_TWIST_CONTROLLER_ACTION_H

#include <ros/ros.h>

#include <std_msgs/Float64.h>
//#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
//#include <nav_msgs/Odometry.h>

//#include <urdf/model.h>
//#include <kdl_parser/kdl_parser.hpp>
//#include <kdl/chainfksolvervel_recursive.hpp>
//#include <kdl/chainiksolvervel_pinv.hpp>
//#include <cob_twist_controller/augmented_solver.h>
//#include <kdl/jntarray.hpp>
//#include <kdl/jntarrayvel.hpp>
//#include <kdl/frames.hpp>

#include <tf/transform_listener.h>

#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <cob_twist_controller_action/TwistControllerActionConfig.h>

class CobTwistControllerAction
{
private:
	ros::NodeHandle nh_;
	std::vector<std::string> joints_;
	unsigned int dof_;
	
public:

	bool initialize();
	void run();
	
	boost::recursive_mutex reconfig_mutex_;
	boost::shared_ptr< dynamic_reconfigure::Server<cob_twist_controller_action::TwistControllerActionConfig> > reconfigure_server_;
	void reconfigure_callback(cob_twist_controller_action::TwistControllerActionConfig &config, uint32_t level);


	
};
#endif
