
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
	
	void broadcast_circle_path();
	void broadcast_linear_path(double*,double*,double*,int);
	void linear_interpolation(double*,double,double,double*,double,double,double*,double, double, double);
	bool drive_homeposition(double,double,double,double);
	void hold_position(double,double,double);
	double betrag(double);
	bool epsilon_area(double,double,double,double);
	Position getEndeffectorPosition();
	void manipulability_Callback(const std_msgs::Float64& msg);
	void timerCallback(const ros::TimerEvent&);
    // Polling
	double update_rate_;


	
private:
	//TF Broadcaster-Var
	tf::TransformBroadcaster br_;
   	tf::Transform transform_;
   	tf::Quaternion q_;
   	tf::TransformListener listener_;
   	tf::StampedTransform stampedTransform_;

	ros::NodeHandle nh_;
	ros::Publisher vis_pub_;
	
	bool homepos_;
	bool reached_home_;
	bool hold_;
	double angle_;
	
	double x_,y_,z_,x_new_,y_new_,z_new_,r_,holdTime_,vel_;
	
	Position pos_;
	int marker_id_;
	int set_markers_;

	
};

#endif

