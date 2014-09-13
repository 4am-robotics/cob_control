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
#include <vector>

void CobArticulation::initialize()
{
	///get params
	if (nh_.hasParam("update_rate"))
	{nh_.getParam("update_rate", update_rate_);}
	else{ update_rate_=68; }
	
	if (nh_.hasParam("path"))
	{nh_.getParam("path", stringPath_);}
	
	if (nh_.hasParam("reference_frame"))
	{nh_.getParam("reference_frame", referenceFrame_);}
	
	if (nh_.hasParam("goal_frame"))
	{nh_.getParam("goal_frame", goalFrame_);}
	
	if (nh_.hasParam("endeffector_frame"))
	{nh_.getParam("endeffector_frame", endeffectorFrame_);}
	
	
	
	
	charPath_ = stringPath_.c_str();
	
	ROS_INFO("...initialized!");
}



void CobArticulation::run()
{	
	ros::Rate r(update_rate_);

	vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	speed_pub_ = nh_.advertise<std_msgs::Float64> ("linear_vel", 1);
	accl_pub_ = nh_.advertise<std_msgs::Float64> ("linear_accl", 1);
	path_pub_ = nh_.advertise<std_msgs::Float64> ("linear_path", 1);
	jerk_pub_ = nh_.advertise<std_msgs::Float64> ("linear_jerk", 1);

	ros::spinOnce();
	load( charPath_ );
}



void CobArticulation::load(const char* pFilename)
{	
	std::vector <double> x_val;
	std::vector <double> y_val;
	std::vector <double> z_val;
	
	TiXmlDocument doc(pFilename);
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
	{
		ROS_INFO("load okay");

		TiXmlHandle docHandle( &doc );
		TiXmlElement* child = docHandle.FirstChild( "Movement" ).FirstChild( "Move" ).ToElement();
		Position p;
		p = getEndeffectorPosition();
		x_=p.x;
		y_=p.y;
		z_=p.z;
		std::string movement;
		
		for( child; child; child=child->NextSiblingElement() )
		{		
			movement = child->Attribute( "move");

			if ("move_lin" == movement){
				ROS_INFO("move_linear");
				x_new_ = atof(child->Attribute( "x"));
				y_new_ = atof(child->Attribute( "y"));
				z_new_ = atof(child->Attribute( "z"));
				vel_ = atof(child->Attribute( "vel"));
				accl_ = atof(child->Attribute( "accl"));
				profile_ = child->Attribute( "profile");
				

				linear_interpolation(&x_val,x_,x_new_,&y_val,y_,y_new_,&z_val,z_,z_new_,vel_,accl_,profile_);
				broadcast_path(&x_val,&y_val,&z_val,roll_,pitch_,yaw_);
			}
			
			if("move_ptp" == movement){
				ROS_INFO("move_ptp");
				x_new_ = atof(child->Attribute( "x"));
				y_new_ = atof(child->Attribute( "y"));
				z_new_ = atof(child->Attribute( "z"));
				roll_ = atof(child->Attribute( "roll"));
				pitch_ = atof(child->Attribute( "pitch"));
				yaw_ = atof(child->Attribute( "yaw"));
				move_ptp(x_new_,y_new_,z_new_,roll_,pitch_,yaw_,0.03);
			}
			
			if("move_circ" == movement){
				ROS_INFO("move_circ");
				
				x_center_ 	= atof(child->Attribute( "x_center"));
				y_center_ 	= atof(child->Attribute( "y_center"));
				z_center_	= atof(child->Attribute( "z_center"));
				r_			= atof(child->Attribute( "r"));
				startAngle_ = atof(child->Attribute( "startangle"));
				endAngle_	= atof(child->Attribute( "endangle"));
				vel_		= atof(child->Attribute( "vel"));
				accl_ 		= atof(child->Attribute( "accl"));
				profile_ 	= child->Attribute( "profile");
				level_ 	= child->Attribute( "level");

				
				circular_interpolation(&x_val,x_center_,&y_val,y_center_,&z_val,z_center_,startAngle_,endAngle_,r_,vel_,accl_, level_,profile_);
				broadcast_path(&x_val,&y_val,&z_val,roll_,pitch_,yaw_);

				x_new_=cos(endAngle_*M_PI/180)*r_;
				y_new_=sin(endAngle_*M_PI/180)*r_;
				z_new_=z_center_;
			}
			
		
			
			if("hold" == movement){
				ROS_INFO("Hold position");
				holdTime_ = atof(child->Attribute( "time"));
				ros::Timer timer = nh_.createTimer(ros::Duration(holdTime_), &CobArticulation::timerCallback, this);
				hold_=true;
				hold_position(x_,y_,z_,roll_,pitch_,yaw_);
			}
			
			x_val.clear();
			y_val.clear();
			z_val.clear();
			
			x_=x_new_;
			y_=y_new_;
			z_=z_new_;
		}	
	}else{
		ROS_WARN("Error loading File");
	}
	
	 
}


void CobArticulation::move_ptp(double x, double y, double z, double roll, double pitch, double yaw, double epsilon){
	reached_pos_=false;
	int reached_pos_counter=0;	
	double ro,pi,ya;
	ros::Rate rate(update_rate_);
	ros::Time now;
	
	
	while(ros::ok()){
		now = ros::Time::now();

		// Linearkoordinaten
		transform_.setOrigin( tf::Vector3(x,y,z) );
	
		// RPY Winkel
		q_.setRPY(roll, pitch, yaw);
		transform_.setRotation(q_);
				
		// Send br Frame
		br_.sendTransform(tf::StampedTransform(transform_, now, referenceFrame_, goalFrame_));

		// Get transformation
		try{
			listener_.waitForTransform(goalFrame_,endeffectorFrame_, now, ros::Duration(0.5));
			listener_.lookupTransform(goalFrame_,endeffectorFrame_, now, stampedTransform_);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}
		
		// Get current RPY out of quaternion
		tf::Quaternion quatern = stampedTransform_.getRotation();
		tf::Matrix3x3(quatern).getRPY(ro,pi,ya);
				
				
		// Wait for arm_7_link to be in position
		if(epsilon_area(stampedTransform_.getOrigin().x(), stampedTransform_.getOrigin().y(), stampedTransform_.getOrigin().z(),ro,pi,ya,epsilon)){
			reached_pos_counter++;	// Count up if end effector position is in the epsilon area to avoid wrong values
		}
		
		if(reached_pos_counter>=50){	
			reached_pos_=true;
		}
		
		if(reached_pos_==true){	// Cancle while loop
			break;
		}
		rate.sleep();
	ros::spinOnce();
	}

}




void CobArticulation::broadcast_path(std::vector <double> *x,std::vector <double> *y, std::vector <double> *z,double roll,double pitch, double yaw){
	ros::Rate rate(update_rate_);
	ros::Time now;

	double T_IPO=pow(update_rate_,-1);
	std_msgs::Float64 msg_vel;
	std_msgs::Float64 msg_accl;
	std_msgs::Float64 msg_path;
	std_msgs::Float64 msg_jerk;
	
	std::vector <double> path;
	std::vector <double> velocity;
	std::vector <double> acceleration;
	std::vector <double> jerk;	
	
//--------------------------------------------------------------------------------------------------------------------------------------------------------	
	// Calculate the path curve
	for(int i=0;i<x->size()-1;i++){
		if(i==0)
			path.push_back(sqrt(pow( (x->at(i+1)-x->at(i)),2) + pow(( y->at(i+1)-y->at(i) ),2) + pow( ( z->at(i+1)-z->at(i) ),2)));	
		else	
			path.push_back(path.at(i-1)+sqrt(pow( (x->at(i+1)-x->at(i)),2) + pow(( y->at(i+1)-y->at(i) ),2) + pow( ( z->at(i+1)-z->at(i) ),2)));
	}
	
	// Calculate the velocity curve
	for(int i=0;i<x->size()-1;i++){
			velocity.push_back(sqrt(pow( (x->at(i+1)-x->at(i)),2) + pow(( y->at(i+1)-y->at(i) ),2) + pow( ( z->at(i+1)-z->at(i) ),2))/T_IPO);	
	}
	// Calculate the acceleration curve
	 for(int i=0;i<velocity.size()-1;i++){
			acceleration.push_back( (velocity.at(i+1) - velocity.at(i))/T_IPO);	
	}
	// Calculate the jerk curve
	 for(int i=0;i<acceleration.size()-1;i++){
			jerk.push_back( (acceleration.at(i+1) - acceleration.at(i))/T_IPO);	
	}
	
//--------------------------------------------------------------------------------------------------------------------------------------------------------	
		
	for(int i=0; i<x->size()-2; i++){
		now = ros::Time::now();
		
		// Linearkoordinaten
		transform_.setOrigin( tf::Vector3( x->at(i),y->at(i),z->at(i) ) );
		
		// RPY Winkel
		q_.setRPY(roll, pitch, yaw);
		transform_.setRotation(q_);   

		marker(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, goalFrame_));
		
		br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, goalFrame_));
		
		// Publish profile data
		msg_path.data = path.at(i);
		msg_vel.data = velocity.at(i);
		msg_accl.data = acceleration.at(i);	
		if(i<jerk.size())
			msg_jerk.data = jerk.at(i);
		
		
		path_pub_.publish(msg_path);
		speed_pub_.publish(msg_vel);
		accl_pub_.publish(msg_accl);
		jerk_pub_.publish(msg_jerk);
		
		ros::spinOnce();
		rate.sleep();
	}
	
}



void CobArticulation::hold_position(double x, double y, double z,double roll,double pitch, double yaw){
	
	ros::Rate rate(update_rate_);
	ros::Time now;

	
	while(hold_){
		now = ros::Time::now();
		
		// Linearkoordinaten
		transform_.setOrigin( tf::Vector3(x,y,z) );
	
		// RPY Winkel
		q_.setRPY(roll, pitch, yaw);
		transform_.setRotation(q_);   	
	
		br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, goalFrame_));
		ros::spinOnce();
		rate.sleep();
	}
}


// Helper Functions 
//--------------------------------------------------------------------------------------------------------------
void CobArticulation::linear_interpolation(std::vector <double> *x,double x_start, double x_end,
										   std::vector <double> *y,double y_start, double y_end,
										   std::vector <double> *z,double z_start, double z_end,
										   double VelMax, double AcclMax, std::string profile) 
{     
	double Se = sqrt(pow((x_end-x_start),2)+pow((y_end-y_start),2)+pow((z_end-z_start),2));
	std::vector<double> pathArray;
	
	// Calculates the Path with Ramp - or Sinoidprofile
	calculateProfile(&pathArray,Se,VelMax,AcclMax,profile);
	
	// Interpolate the linear path
	for(int i=0;i<pathArray.size();i++){	
		x->push_back(x_start + pathArray.at(i) * (x_end-x_start)/Se);
		y->push_back(y_start + pathArray.at(i) * (y_end-y_start)/Se);
		z->push_back(z_start + pathArray.at(i) * (z_end-z_start)/Se);	
	}
}



void CobArticulation::circular_interpolation(std::vector<double>* x,double x_mittel,
										     std::vector<double>* y,double y_mittel, 
										     std::vector<double>* z,double z_mittel,
										     double startAngle, double endAngle,
										     double r, double VelMax, double AcclMax, std::string level, std::string profile)  
{
	startAngle=startAngle*M_PI/180;
	endAngle=endAngle*M_PI/180;
	
	double Se = endAngle-startAngle;
	bool forward;
	
	if(Se < 0)
		forward=false;
	else
		forward=true;
		
	Se = betrag(Se);
		
	std::vector<double> pathArray;
	
	// Calculates the Path with Ramp - or Sinoidprofile
	calculateProfile(&pathArray,Se,VelMax,AcclMax,profile);
		
		
	// Interpolate the circular path
	if(level=="XY"){
		if(forward){
			for(int i=0;i<pathArray.size();i++){	
				x->push_back(x_mittel+cos(pathArray.at(i))*r);
				y->push_back(y_mittel+sin(pathArray.at(i))*r);
				z->push_back(z_mittel);
			}
		}
		else{
			for(int i=0;i<pathArray.size();i++){	
				x->push_back(x_mittel+cos(startAngle-pathArray.at(i))*r);
				y->push_back(y_mittel+sin(startAngle-pathArray.at(i))*r);
				z->push_back(z_mittel);
			}
		}
	}
	
	if(level=="XZ"){
		if(forward){
			for(int i=0;i<pathArray.size();i++){	
				x->push_back(x_mittel+cos(pathArray.at(i))*r);
				y->push_back(y_mittel);
				z->push_back(z_mittel+sin(pathArray.at(i))*r);
			}
		}
		else{
			for(int i=0;i<pathArray.size();i++){	
				x->push_back(x_mittel+cos(startAngle-pathArray.at(i))*r);
				y->push_back(y_mittel);
				z->push_back(z_mittel+sin(startAngle-pathArray.at(i))*r);
			}
		}
	}
	
	if(level=="YZ"){
		if(forward){
			for(int i=0;i<pathArray.size();i++){	
				x->push_back(x_mittel);
				y->push_back(y_mittel+cos(pathArray.at(i))*r);
				z->push_back(z_mittel+sin(pathArray.at(i))*r);
			}
		}
		else{
			for(int i=0;i<pathArray.size();i++){	
				x->push_back(x_mittel);
				y->push_back(y_mittel+cos(startAngle-pathArray.at(i))*r);
				z->push_back(z_mittel+sin(startAngle-pathArray.at(i))*r);
			}
		}
	}
}


void CobArticulation::calculateProfile(std::vector<double> *pathArray,double Se, double VelMax, double AcclMax, std::string profile){
	
	int steps_te,steps_tv,steps_tb=0;
	double tv,tb,te=0;
	double T_IPO=pow(update_rate_,-1);

	

	if(profile == "ramp"){
		// Calculate the Ramp Profile Parameters
		if (VelMax > sqrt(Se*AcclMax)){
			VelMax = sqrt(Se*AcclMax);
		}
		tb = VelMax/AcclMax;
		te = (Se / VelMax) + tb;
		tv = te - tb;
	}
	else{
		// Calculate the Sinoide Profile Parameters
		if (VelMax > sqrt(Se*AcclMax/2)){
			VelMax = sqrt(Se*AcclMax/2);
		}
		tb = 2*VelMax/AcclMax;
		te = (Se / VelMax) + tb;
		tv = te - tb;		
	}

	// Interpolationsteps for every timesequence
	steps_tb = (double)tb / T_IPO;
	steps_tv = (double)(tv-tb) / T_IPO;
	steps_te = (double)(te-tv) / T_IPO;


	// Reconfigure timings wtih T_IPO
	tb=steps_tb*T_IPO;
	tv=(steps_tb+steps_tv)*T_IPO;
	te=(steps_tb+steps_tv+steps_te)*T_IPO;
	
			
	ROS_INFO("Vm: %f m/s",VelMax);
	ROS_INFO("Bm: %f m/s²",AcclMax);
	ROS_INFO("Se: %f ",Se);
	ROS_INFO("tb: %f s",tb);
	ROS_INFO("tv: %f s",tv);
	ROS_INFO("te: %f s",te);
	

	if(profile == "ramp"){
		ROS_INFO("Ramp Profile");
		// Calculate the ramp profile path
		// 0 <= t <= tb
		for(int i=0;i<=steps_tb-1;i++){	
			pathArray->push_back(0.5*AcclMax*pow((T_IPO*i),2));
		}
		// tb <= t <= tv
		for(int i=steps_tb;i<=(steps_tb+steps_tv-1);i++){	
			pathArray->push_back(VelMax*(T_IPO*i)-0.5*pow(VelMax,2)/AcclMax);
		}
		// tv <= t <= te
		for(int i=(steps_tb+steps_tv);i<(steps_tv+steps_tb+steps_te-1);i++){	
			//pathArray[i] = Se - 0.5*AcclMax* pow(((steps_tb+steps_tv+steps_te-i)*T_IPO),2);
			pathArray->push_back(VelMax * (te-tb) - 0.5*AcclMax* pow(te-(i*T_IPO),2));
		}
	}
	else{
		ROS_INFO("Sinoide Profile");
		// Calculate the sinoide profile path
		// 0 <= t <= tb
		for(int i=0;i<=steps_tb-1;i++){	
			pathArray->push_back(  AcclMax*(0.25*pow(i*T_IPO,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*T_IPO))-1))  );
		}
		// tb <= t <= tv
		for(int i=steps_tb;i<=(steps_tb+steps_tv-1);i++){	
			pathArray->push_back(VelMax*(i*T_IPO-0.5*tb));
		}
		// tv <= t <= te
		for(int i=(steps_tb+steps_tv);i<(steps_tv+steps_tb+steps_te-1);i++){	
			pathArray->push_back(0.5*AcclMax*( te*(i*T_IPO + tb)  -  0.5*(pow(i*T_IPO,2)+pow(te,2)+2*pow(tb,2))  + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos( ((2*M_PI)/tb) * (i*T_IPO-tv)))));
		}
	}
	
	
}



double CobArticulation::betrag(double num){
	if(num<0){
		num=num*(-1.0);
	}
	return num;
}



CobArticulation::Position CobArticulation::getEndeffectorPosition()
{		
	for(int i=0;i<3;i++){
		ros::Time now = ros::Time::now();;
		// Get transformation
		try{
			listener_.waitForTransform(referenceFrame_,endeffectorFrame_, now, ros::Duration(0.5));
			listener_.lookupTransform(referenceFrame_,endeffectorFrame_, now, stampedTransform_);
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
bool CobArticulation::epsilon_area(double x,double y, double z, double roll, double pitch, double yaw,double epsilon)
{
	bool x_okay=false, y_okay=false, z_okay=false;
	bool roll_okay=false, pitch_okay=false, yaw_okay=false;
	
	x=betrag(x);
	y=betrag(y);
	z=betrag(z);
	roll=betrag(roll);
	pitch=betrag(pitch);
	yaw=betrag(yaw);
	
	if(x < epsilon){ x_okay = true; };
	if(y < epsilon){ y_okay = true; };
	if(z < epsilon){ z_okay = true; };
	if(roll < epsilon){ roll_okay = true; };
	if(pitch < epsilon){ pitch_okay = true; };
	if(yaw < epsilon){ yaw_okay = true; };
	
	if(x_okay && y_okay && z_okay && roll_okay && pitch_okay && yaw_okay){
		return true;
	}else{
		return false;
	}
}




void CobArticulation::marker(tf::StampedTransform tf)
{
  	visualization_msgs::Marker marker;
	marker.header.frame_id = referenceFrame_;
	marker.header.stamp = ros::Time();
	marker.ns = "cob_articulation";
	marker.id = marker_id_;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
		
	marker.pose.position.x = tf.getOrigin().x();
	marker.pose.position.y = tf.getOrigin().y();
	marker.pose.position.z = tf.getOrigin().z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	
	marker.color.a = 1.0;

	marker_id_++;
	vis_pub_.publish( marker );
	
}

void CobArticulation::timerCallback(const ros::TimerEvent& event){
	hold_=false;
}

