
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
 *   ROS package name: cob_articulation
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de
 *
 * \date Date of creation: August, 2014
 *
 * \brief
 *   ...
 *
 ****************************************************************/
#ifndef COB_ARTICULATION_H
#define COB_ARTICULATION_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tinyxml.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <vector>


class CobArticulation
{
public:
	struct Position
    {
   		double x,y,z,alpha,beta,gamma;
	};

	CobArticulation() {;}
	
	void initialize();
	void run();
	void load(const char*);
	
	// Main functions
	void broadcast_path(std::vector<double>*,std::vector<double>*,std::vector<double>*,double,double,double);
	void linear_interpolation(std::vector<double>*,double,double,std::vector<double>*,double,double,std::vector<double>*,double,double,double,double,std::string);
	void circular_interpolation(std::vector<double>*,double,std::vector<double>*,double,std::vector<double>*,double,double,double,double,double,double,std::string,std::string);
	void move_ptp(double,double,double,double,double,double,double);
	void hold_position(double,double,double,double,double,double);
	
	// Helper function
	double betrag(double);
	bool epsilon_area(double,double,double,double,double,double,double);
	Position getEndeffectorPosition();
	void marker(tf::StampedTransform);
	void timerCallback(const ros::TimerEvent&);
	void calculateProfile(std::vector<double>*,double, double, double,std::string);
	
private:
	ros::NodeHandle nh_;
	
	// Publisher
	ros::Publisher  vis_pub_;
	ros::Publisher	path_pub_;
	ros::Publisher	speed_pub_;
	ros::Publisher	accl_pub_;

	//TF Broadcaster-Var
	tf::TransformBroadcaster br_;
   	tf::Transform transform_;
   	tf::Quaternion q_;
   	tf::TransformListener listener_;
   	tf::StampedTransform stampedTransform_;

	// Var for XML Parser
	double x_,y_,z_,x_new_,y_new_,z_new_,x_center_,y_center_,z_center_,r_,holdTime_,vel_,accl_,startAngle_,endAngle_,roll_,pitch_,yaw_;
	std::string profile_,level_;
	
	// Var for PTP Movement and hold Position
	bool reached_pos_,hold_;
	
	// For endeffector Postion
	Position pos_;
	
	// Update Rate
	double update_rate_;
	
};

#endif

