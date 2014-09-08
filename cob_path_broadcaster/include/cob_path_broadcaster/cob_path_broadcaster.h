
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
 *   ROS package name: cob_path_broadcaster
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
#ifndef COB_PATH_BROADCASTER_H
#define COB_PATH_BROADCASTER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>


class CobPathBroadcaster
{
public:
	CobPathBroadcaster() {;}
	~CobPathBroadcaster();
	
	void initialize();
	void run();
	void broadcast_circle_path();
	void interpolate_linear(double,double,double[],int);
	bool drive_homeposition(double,double,double,double);
	double betrag(double);
	bool epsilon_area(double,double,double,double);
	void manipulability_Callback(const std_msgs::Float64& msg);



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


	// Circle
	bool fwd;
	double r_;
	double offset_x_;
	double offset_y_;
	double offset_z_;
	
	double start_angle_;
	double end_angle_;
	double angle_;
	
	std::string circle_level_;
	bool reached_home_;
	ros::Publisher vis_pub_;
	
	int marker_id_;
	int set_markers_;
};

#endif

