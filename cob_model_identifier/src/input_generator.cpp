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
#include <ros/ros.h>
#include <vector>
#include <cob_model_identifier/input_generator.h>


void InputGenerator::initialize()
{
	///get params
	if (nh_.hasParam("trans_x"))
	{
		nh_.getParam("trans_x",trans_x_);
	}
	
	
	if (nh_.hasParam("trans_y"))
	{
		nh_.getParam("trans_y",trans_y_);
	}
	
	
	if (nh_.hasParam("trans_z"))
	{
		nh_.getParam("trans_z",trans_z_);
	}

	if (nh_.hasParam("rot_x"))
	{
		nh_.getParam("rot_x",rot_x_);
	}
	
	if (nh_.hasParam("rot_y"))
	{
		nh_.getParam("rot_y",rot_y_);
	}
	
	if (nh_.hasParam("rot_z"))
	{
		nh_.getParam("rot_z",rot_z_);
	}	
	ROS_INFO("...initialized input generator!");
}


void InputGenerator::run()
{
	ros::Rate r(100.0);
	twist_pub_ = nh_.advertise<geometry_msgs::Twist> ("/arm_controller/command_twist", 1);
	wait1sec_=false;
	wait10sec_=false;

	for(int i=0;i<100;i++){
		publish_twist(true);
		ros::spinOnce();
		r.sleep();
	}

	timer2 = nh_.createTimer(ros::Duration(6.0), &InputGenerator::timerCallback10sec, this,true);
	while(!wait10sec_){
		publish_twist(false);
		ros::spinOnce();
		r.sleep();
	}

}

void InputGenerator::publish_twist(bool sendZero)
{
	geometry_msgs::Twist twist_msg;
	
	if(sendZero){
		twist_msg.linear.x = 0;
		twist_msg.linear.y = 0;
		twist_msg.linear.z = 0;

		twist_msg.angular.x = 0;
		twist_msg.angular.y = 0;
		twist_msg.angular.z = 0;
	}
	else{
		twist_msg.linear.x = trans_x_;
		twist_msg.linear.y = trans_y_;
		twist_msg.linear.z = trans_z_;
		
		twist_msg.angular.x = rot_x_;
		twist_msg.angular.y = rot_y_;
		twist_msg.angular.z = rot_z_;
		
		pbrs_counter++;
	}
	twist_pub_.publish(twist_msg);
}


void InputGenerator::timerCallback1sec(const ros::TimerEvent& event){
	wait1sec_=true;
}

void InputGenerator::timerCallback10sec(const ros::TimerEvent& event){
	wait10sec_=true;
}


