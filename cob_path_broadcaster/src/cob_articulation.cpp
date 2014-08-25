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
#include <cob_path_broadcaster/cob_articulation.h>
#include <tf/transform_broadcaster.h>
#include <tinyxml.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

void CobArticulation::initialize()
{
	///get params
	if (nh_.hasParam("update_rate"))
	{	nh_.getParam("update_rate", update_rate_);	}


	update_rate_=68;
	homepos_=false;
	set_markers_=0;
	marker_id_=0;
	ROS_INFO("...initialized!");
}



void CobArticulation::run()
{	
	
	ros::Time time = ros::Time::now();
	ros::Time last_update_time = time;
	ros::Duration period = time - last_update_time;
	
	ros::Rate r(update_rate_);

	vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	ros::Subscriber sub = nh_.subscribe("/arm_controller/manipulability2", 5, &CobArticulation::manipulability_Callback,this);
	ros::spinOnce();
	load( "/home/fxm-cm/catkin_ws/src/cob_control/cob_path_broadcaster/movement/move.prog" );
	ROS_INFO("load okay");
}



void CobArticulation::load(const char* pFilename)
{
	int interpolation_elements;
	
	TiXmlDocument doc(pFilename);
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
	{
			
		TiXmlHandle hDoc(&doc);
		TiXmlHandle docHandle( &doc );
		TiXmlElement* child = docHandle.FirstChild( "Movement" ).FirstChild( "Move" ).ToElement();
		Position p;
		
		std::string movement;
		
		for( child; child; child=child->NextSiblingElement() )
		{		
			x_new_=y_new_=z_new_=vel_=0;
			movement = child->Attribute( "move");

			if ("move_lin" == movement){
				ROS_INFO("move_linear");
				x_new_ = atof(child->Attribute( "x"));
				y_new_ = atof(child->Attribute( "y"));
				z_new_ = atof(child->Attribute( "z"));
				vel_ = atof(child->Attribute( "speed"));
				p = getEndeffectorPosition();

		
				double x_arr[(int)(vel_*update_rate_)];
				double y_arr[(int)(vel_*update_rate_)];
				double z_arr[(int)(vel_*update_rate_)];		
				//double *x_arr = malloc(sizeof(double) * (int)(vel_*update_rate_));

				
				linear_interpolation(x_arr,p.x,x_new_,y_arr,p.y,y_new_,z_arr,p.z,z_new_,vel_);
				broadcast_linear_path(x_arr,y_arr,z_arr,vel_*update_rate_);
				
			}
			
			if("move_ptp" == movement){
				ROS_INFO("move_ptp");
				x_new_ = atof(child->Attribute( "x"));
				y_new_ = atof(child->Attribute( "y"));
				z_new_ = atof(child->Attribute( "z"));
				drive_homeposition(x_new_,y_new_,z_new_,0.03);
			}
			
			if("hold" == movement){
				ROS_INFO("Hold position");
				holdTime_ = atof(child->Attribute( "time"));
				ros::Timer timer = nh_.createTimer(ros::Duration(holdTime_), &CobArticulation::timerCallback, this);
				hold_=true;
				hold_position(x_,y_,z_);
			}
			x_=x_new_;
			y_=y_new_;
			z_=z_new_;
		}
		
			
		
	}else{
		ROS_WARN("Error loading File");
	}
	
	
}


bool CobArticulation::drive_homeposition(double x_home, double y_home, double z_home, double epsilon){
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
	ros::spinOnce();
	}
	
	return true;
}




void CobArticulation::broadcast_linear_path(double*x,double*y, double*z,int elements){
	ros::Rate rate(update_rate_);
	ros::Time now;

ROS_INFO("Abfahrt");

	for(int i=0; i<elements; i++){
		now = ros::Time::now();
		
		// Linearkoordinaten
		transform_.setOrigin( tf::Vector3(x[i],y[i],z[i]) );
	
		// RPY Winkel
		q_.setRPY(0, 0, 0);
		transform_.setRotation(q_);   

		br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "world", "br"));
		ros::spinOnce();
		rate.sleep();
	}
ROS_INFO("Ende");
}



void CobArticulation::hold_position(double x, double y, double z){
	
	ros::Rate rate(update_rate_);
	ros::Time now;

	
	while(hold_){
		now = ros::Time::now();
		
		// Linearkoordinaten
		transform_.setOrigin( tf::Vector3(x,y,z) );
	
		// RPY Winkel
		q_.setRPY(0, 0, 0);
		transform_.setRotation(q_);   	
	
		br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "world", "br"));
		ros::spinOnce();
		rate.sleep();
	}
}


// Helper Functions 
//--------------------------------------------------------------------------------------------------------------

void CobArticulation::linear_interpolation(double* x,double x_start, double x_end,
										   double* y,double y_start, double y_end,
										   double* z,double z_start, double z_end,
										   double vel) {     

double Se = sqrt(pow((x_end-x_start),2)+pow((y_end-y_start),2)+pow((z_end-z_start),2));
int N = vel*update_rate_;
double segment = Se/N;

	for(int i=0;i<N;i++){	
		x[i] = x_start + segment*(i+1) * (x_end-x_start)/Se;
		y[i] = y_start + segment*(i+1) * (y_end-y_start)/Se;
		z[i] = z_start + segment*(i+1) * (z_end-z_start)/Se;	
	}
	ROS_INFO("Se: %f",Se);
}



double CobArticulation::betrag(double num){
	if(num<0){
		num=num*(-1.0);
	}
	return num;
}



CobArticulation::Position CobArticulation::getEndeffectorPosition(){
		
		for(int i=0;i<3;i++){
		ros::Time now = ros::Time::now();;
		// Get transformation
		try{
			listener_.waitForTransform("/arm_base_link","/arm_7_link", now, ros::Duration(0.5));
			listener_.lookupTransform("/arm_base_link","/arm_7_link", now, stampedTransform_);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}	
	}
		pos_.x=stampedTransform_.getOrigin().x();
		pos_.y=stampedTransform_.getOrigin().y();
		pos_.z=stampedTransform_.getOrigin().z();
				
		return pos_;
}



// Checks if the endeffector is in the area of the 'br' frame
bool CobArticulation::epsilon_area(double x,double y, double z,double epsilon){
	bool x_okay=false, y_okay=false, z_okay=false;
	
	x=betrag(x);
	y=betrag(y);
	z=betrag(z);
	
	
	if(x < epsilon){ x_okay = true; };
	if(y < epsilon){ y_okay = true; };
	if(z < epsilon){ z_okay = true; };

	
	if(x_okay && y_okay && z_okay){
		return true;
	}else{
		return false;
	}
}




void CobArticulation::manipulability_Callback(const std_msgs::Float64& msg)
{
  	visualization_msgs::Marker marker;
	marker.header.frame_id = "arm_7_link";
	marker.header.stamp = ros::Time();
	marker.ns = "cob_articulation";
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
	
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;
	
	
	marker.color.a = 1.0;

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

void CobArticulation::timerCallback(const ros::TimerEvent& event){
	hold_=false;
}
