
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

	
	struct Path
    {
   		std::vector<double> x;
   		std::vector<double> y;
   		std::vector<double> z;
   		std::vector<double> roll;
   		std::vector<double> pitch;
   		std::vector<double> yaw;
	};

	
	struct WayPoint
    {
   		double x,y,z,roll,pitch,yaw;
	};

	CobArticulation() {;}
	
	void initialize();
	void run();
	void load(const char*);
	
	void broadcast_path(std::vector<double>*,std::vector<double>*,std::vector<double>*);
	void linear_interpolation(std::vector<double>*,double,double,std::vector<double>*,double,double,std::vector<double>*,double,double,double,double,std::string);
	void circular_interpolation(std::vector<double>*,double,std::vector<double>*,double,std::vector<double>*,double,double,double,double,double,double,std::string);
	//Path circular_interpolation(CobArticulation::WayPoint,double,double,double,double,double,std::string);
	bool move_ptp(double,double,double,double);
	void hold_position(double,double,double);
	double betrag(double);
	bool epsilon_area(double,double,double,double);
	Position getEndeffectorPosition();
	void manipulability_Callback(const std_msgs::Float64& msg);
	void timerCallback(const ros::TimerEvent&);
	void calculateProfile(std::vector<double>*,double, double, double,std::string);
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
	
	double x_,y_,z_,x_new_,y_new_,z_new_,x_center_,y_center_,z_center_,r_,holdTime_,vel_,accl_,startAngle_,endAngle_;
	std::string profile_;
	
	Position pos_;
	int marker_id_;
	int set_markers_;

	
};

#endif

