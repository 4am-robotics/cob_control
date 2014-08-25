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
#include <visualization_msgs/Marker.h>



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
	
	set_markers_=0;
	marker_id_=0;
	
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
	
	vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
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
		set_markers_++; // Stoping the marker publisher
	}
	
	if(angle_<=start_angle_ && fwd==false){
		fwd=true;
	}

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
	if(set_markers_<1){
  	visualization_msgs::Marker marker;
	marker.header.frame_id = "arm_7_link";
	marker.header.stamp = ros::Time();
	marker.ns = "cob_path_broadcaster";
	marker.id = marker_id_;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	
	
	
	marker.pose.position.x = stampedTransform_.getOrigin().x();
	marker.pose.position.y = stampedTransform_.getOrigin().y();
	marker.pose.position.z = stampedTransform_.getOrigin().z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	
	marker.scale.x = 0.007;
	marker.scale.y = 0.007;
	marker.scale.z = 0.007;
	
	
	marker.color.a = 1.0;
	/*
	// Transform YMI into RGB Colors 
	int color = msg.data * 10000000;// / 0xFFFFFF00;
	int normalized_color = color * (16777215/110000);
	
	ROS_INFO("normalized: %i",normalized_color);
	
	uint8_t red 	= 	(normalized_color & 0xFF0000) >> 16;
	uint8_t green 	=	(normalized_color & 0x00FF00) >> 8;
	uint8_t blue 	= 	(normalized_color & 0x0000FF);
	
	ROS_INFO("red   %i",red);
	ROS_INFO("green %i",green);
	ROS_INFO("blue  %i",blue);
	
	float red_normalized = ((float)red) * 0.003921569;
	float green_normalized = ((float)green) * 0.003921569;
	float blue_normalized = ((float)blue) * 0.003921569;
		
	ROS_INFO("red_norm   %f",red_normalized);
	ROS_INFO("green_norm %f",green_normalized);
	ROS_INFO("blue_norm  %f",blue_normalized);
	
	//uint8_t test = (16777215 & 0xFF0000)>>20;
	//ROS_INFO("16777215 & 0xFF0000 = %i",test);
				//1111 1111 0000 0000 0000 0000
				//1111 1111 1111 1111 1111 1111
				//1111 1111 0000 0000 0000 0000
	marker.color.r = red_normalized;
	marker.color.g = green_normalized;
	marker.color.b = blue_normalized;
	*/
	
	float value = msg.data * 9.090909091;


		if (0 <= value && value <= 0.125) {
			marker.color.r = 4*value + .5; 
			marker.color.g = 0;
			marker.color.b = 0; 
			
		} else if (0.125 < value && value <= 0.375) {
			marker.color.r = 1;
			marker.color.g = 0; 
			marker.color.b = 4*value - .5;
			
		} else if (0.375 < value && value <= 0.625) {
			marker.color.r = -4*value + 2.5;;
			marker.color.g = 0;
			marker.color.b = 0;
			
		}else if (0.625 < value && value <= 1) {
			marker.color.r = 0;
			marker.color.g = 4*value + .5;
			marker.color.b = 0;
		}

	
	
	marker_id_++;
	vis_pub_.publish( marker );
	}
}



