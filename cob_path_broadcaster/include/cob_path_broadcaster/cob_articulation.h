
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
	void initialize();
	void run();
	void load(const char*);
	
	// Main functions
	void broadcast_path(std::vector<double>*,std::vector<double>*,std::vector<double>*,std::vector<double>*,std::vector<double>*,std::vector<double>*,double,double,std::string);	
	void pose_path_broadcaster(std::vector <geometry_msgs::Pose> *);
	void linear_interpolation(std::vector <geometry_msgs::Pose> *poseVector,geometry_msgs::Pose, geometry_msgs::Pose,double,double,std::string,bool); 			
	void circular_interpolation_any_level(std::vector<geometry_msgs::Pose>*,double,double,double,double,double,double,double,double,double,double,double,std::string);																							
	void move_ptp(double,double,double,double,double,double,double);
	void hold_position(geometry_msgs::Pose);
	
	// Helper function
	double betrag(double);
	bool epsilon_area(double,double,double,double,double,double,double);
	geometry_msgs::Pose getEndeffectorPosition();
	void marker(tf::StampedTransform,int,double,double,double,std::string);
	void showDot(double,double,double,int,double,double,double,std::string);
	void showLevel(tf::Transform,int,double,double,double,std::string);
	void timerCallback(const ros::TimerEvent&);
	void calculateProfile(std::vector<double>*,double,double,double,std::string);
	void calculateProfileForAngularMovements(std::vector<double> *,double,double,double,double,double,double,std::string,bool);
	void generatePath(std::vector<double>*,double,double,double,double,int,std::string);
	void start_tracking();
	void stop_tracking();
	

	
private:
	ros::NodeHandle nh_;
	
	// Publisher
	ros::Publisher  vis_pub_;
	ros::Publisher	path_pub_;
	ros::Publisher	speed_pub_;
	ros::Publisher	accl_pub_;
	ros::Publisher	jerk_pub_;
	
	//TF Broadcaster-Var
	tf::TransformBroadcaster br_;
   	tf::Transform transform_;
   	tf::Quaternion q_;
   	tf::TransformListener listener_;
   	tf::StampedTransform stampedTransform_;

	// Var for XML Parser
	double x_,y_,z_,x_new_,y_new_,z_new_,x_center_,y_center_,z_center_,r_,holdTime_,vel_,accl_,startAngle_,endAngle_,roll_,pitch_,yaw_,quat_x_,quat_y_,quat_z_,quat_w_;
	std::string profile_,level_,fixAxis_,justRotate_;
	
	// Var for PTP Movement and hold Position
	bool reached_pos_,hold_;
	
	
	// yaml params
	double update_rate_;
	std::string stringPath_,referenceFrame_,goalFrame_,endeffectorFrame_;
	const char* charPath_;
	
	
	int marker1_,marker2_;
};

#endif

