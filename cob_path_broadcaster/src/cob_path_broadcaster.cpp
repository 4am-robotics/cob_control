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
 *   Author: Christoph Mark, email: Christoph.Mark@ipa.fraunhofer.de
 *
 * \date Date of creation: August, 2014
 *
 * \brief
 *   ...
 *
 ****************************************************************/
#include <ros/ros.h>
#include <math.h>
#include <cob_path_broadcaster/cob_path_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>



void CobPathBroadcaster::initialize()
{
	///get params
	if (nh_.hasParam("update_rate"))
	{	nh_.getParam("update_rate", update_rate_);	}
	else
	{	update_rate_ = 20.0;	}	
	
	if (nh_.hasParam("radius"))
	{	nh_.getParam("radius", r_);	}
	else
	{	r_ = 0.5;	}	
	
	if (nh_.hasParam("offset_x"))
	{	nh_.getParam("offset_x", offset_x_);	}
	else
	{	offset_x_ = 0.0;	}	
	
	if (nh_.hasParam("offset_y"))
	{	nh_.getParam("offset_y", offset_y_);	}
	else
	{	offset_y_ = 0.0;	}	
	
	
	if (nh_.hasParam("offset_z"))
	{	nh_.getParam("offset_z", offset_z_);	}
	else
	{	offset_z_ = 0.50;	}	

	if (nh_.hasParam("start_angle"))
	{	nh_.getParam("start_angle", start_angle_);	}
	else
	{	start_angle_ = 0.0;	}	
	
	
	if (nh_.hasParam("end_angle"))
	{	nh_.getParam("end_angle", end_angle_);}
	else
	{	end_angle_ = 360.0;	}	
	
		if (nh_.hasParam("circle_level"))
	{	nh_.getParam("circle_level", circle_level_);}
	else
	{	circle_level_ = "xy";	}
	
	angle_ = start_angle_*M_PI/180;
	end_angle_=end_angle_*M_PI/180;
	start_angle_=start_angle_*M_PI/180;
	fwd=true;
	
	
	ROS_INFO("...initialized!");
}


void CobPathBroadcaster::run()
{
	bool homepos=false;
	ros::Time time = ros::Time::now();
	ros::Time last_update_time = time;
	ros::Duration period = time - last_update_time;
	
	ros::Rate r(update_rate_);
	ros::NodeHandle nh_;
	
	//ros::Publisher vis_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	ros::Subscriber sub = nh_.subscribe("/arm_controller/manipulability2", 5, &CobPathBroadcaster::manipulability_Callback,this);


	// Homeposition
	if(circle_level_=="xy"){	// x-y level
			homepos=drive_homeposition(offset_x_+cos(start_angle_)*r_, offset_y_+sin(start_angle_)*r_,offset_z_,0.03);
	}
	if(circle_level_=="xz"){	// x-z level
			homepos=drive_homeposition(offset_x_+cos(start_angle_)*r_, offset_y_, offset_z_+sin(start_angle_)*r_,0.03);
	}	
	if(circle_level_=="yz"){ 	// y-z level
			homepos=drive_homeposition(offset_x_, offset_y_+cos(start_angle_)*r_, offset_z_+sin(start_angle_)*r_,0.03) ;
	}


	while(ros::ok())
	{
		
		if(homepos){
			//Call circle broadcast
			broadcast_circle_path();
		}
		
		
		
		ros::spinOnce();
		r.sleep();
	}
}



// Circle
void CobPathBroadcaster::broadcast_circle_path()
{
	const double degree = M_PI/180;
	
		// Linearkoordinaten
		if(circle_level_=="xy"){	// Circlepath in x-y level
				transform_.setOrigin( tf::Vector3(offset_x_+cos(angle_)*r_, offset_y_+sin(angle_)*r_, offset_z_) );
		}
		if(circle_level_=="xz"){	// Circlepath in x-z level
				transform_.setOrigin( tf::Vector3(offset_x_+cos(angle_)*r_, offset_y_, offset_z_+sin(angle_)*r_) );
		}	
		if(circle_level_=="yz"){ 	// Circlepath in y-z level
				transform_.setOrigin( tf::Vector3(offset_x_, offset_y_+cos(angle_)*r_, offset_z_+sin(angle_)*r_) );
		}
	
		// RPY Winkel
		q_.setRPY(0, 0, 0);
		transform_.setRotation(q_);
    
    if(fwd){
		angle_ += degree/4;
	}else{
		angle_ -= degree/4;
	}
	
	if(angle_>=end_angle_){
		//angle_=start_angle_;
		fwd=false;
	}
	
	if(angle_<=start_angle_ && fwd==false){
		fwd=true;
	}
	ROS_INFO("start_angle: %f end_angle: %f angle: %f",start_angle_,end_angle_,angle_);

	br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "world", "br"));
}

void CobPathBroadcaster::interpolate_linear(double start, double end, double ret[], int elements){
	
	double length = end - start;
	double segment = length/elements;
	
	for(int i=0;i<elements;i++){
		ret[i]=segment*i;
	}
	
}



bool CobPathBroadcaster::drive_homeposition(double x_home, double y_home, double z_home, double epsilon){
	reached_home_=false;
	int reached_home_counter=0;	
	ros::Rate rate(update_rate_);
	ros::Time now;

	
	
	while(ros::ok()){
		now = ros::Time::now();
		
		// Linearkoordinaten
		transform_.setOrigin( tf::Vector3(x_home,y_home,z_home) );
	
		// RPY Winkel
		q_.setRPY(0, 0, 0);
	
		transform_.setRotation(q_);
				
		// Send br Frame
		br_.sendTransform(tf::StampedTransform(transform_, now, "world", "br"));

		// Get transformation
		try{
			listener_.waitForTransform("/br","/arm_7_link", now, ros::Duration(0.5));
			listener_.lookupTransform("/br","/arm_7_link", now, stampedTransform_);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}
		
		// Wait for arm_7_link to be in position
		if(epsilon_area(stampedTransform_.getOrigin().x(), stampedTransform_.getOrigin().y(), stampedTransform_.getOrigin().z(), epsilon)){
			reached_home_counter++;	// Count up if end effector position is in the epsilon area to avoid wrong values
		}
		
		if(reached_home_counter>=50){	
			reached_home_=true;
		}
		
		if(reached_home_==true){	// Cancle while loop
			break;
		}
		rate.sleep();
	}
	ROS_INFO("...Home!");
	return true;
}

// Checks if the endeffector is in the area of the 'br' frame
bool CobPathBroadcaster::epsilon_area(double x,double y, double z,double epsilon){
	bool x_okay=false, y_okay=false, z_okay=false;
	
	x=betrag(x);
	y=betrag(y);
	z=betrag(z);
	
	
	if(x < epsilon){ x_okay = true; };
	if(y < epsilon){ y_okay = true; };
	if(z < epsilon){ z_okay = true; };
	ROS_INFO("X:   %f < %f",x,epsilon);
	ROS_INFO("Y:   %f < %f",y,epsilon);
	ROS_INFO("Z:   %f < %f",z,epsilon);


	
	if(x_okay && y_okay && z_okay){
		return true;
	}else{
		return false;
	}
}

double CobPathBroadcaster::betrag(double num){
	if(num<0){
		num=num*(-1.0);
	}
	return num;
}



void CobPathBroadcaster::manipulability_Callback(const std_msgs::Float64& msg)
{
	/*
  	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	
	
	marker.pose.position.x = transform_.getOrigin().x();
	marker.pose.position.y = transform_.getOrigin().y();
	marker.pose.position.z = transform_.getOrigin().z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	vis_pub.publish( marker );
	*/
}



