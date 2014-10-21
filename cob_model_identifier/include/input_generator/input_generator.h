
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
 *   ROS package name: cob_model_identifier
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de
 *
 * \date Date of creation: September, 2014
 *
 * \brief
 *   ...
 *
 ****************************************************************/
#ifndef INPUTGENERATOR_H
#define INPUTGENERATOR_H

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>

class InputGenerator
{
public:
	void initialize();
	void run();

	void publish_twist(bool sendZero);
	void timerCallback1sec(const ros::TimerEvent& event);
	void timerCallback10sec(const ros::TimerEvent& event);
	void sweep(double f_start, double f_end, double interval, int n_steps);

private:
	ros::NodeHandle nh_;
	ros::Publisher twist_pub_;
	std::string axis_;
	bool wait1sec_;
	bool wait10sec_;
	double calls,signal;
	double trans_x_,trans_y_,trans_z_;
	double rot_x_,rot_y_,rot_z_;
	ros::Timer timer2,timer;
	std::vector<double> pbrs;
	int pbrs_counter;
};

#endif

