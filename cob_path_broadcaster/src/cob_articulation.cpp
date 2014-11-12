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
#include <ros/ros.h>
#include <math.h>
#include <cob_path_broadcaster/cob_articulation.h>
#include <tf/transform_broadcaster.h>
#include <tinyxml.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <kdl_conversions/kdl_msg.h>
#include <std_srvs/Empty.h>
#include <cob_srvs/SetString.h>
#include <algorithm>
#include "ros/package.h"


bool CobArticulation::initialize()
{
	bool success=true;
	///get params
	if (nh_.hasParam("update_rate"))
	{
		nh_.getParam("update_rate", update_rate_);
	}
	else{ update_rate_=68; }
	
	if (nh_.hasParam("file_name"))
	{
		nh_.getParam("file_name", fileName_);
	}else{ success = false; }
	
	if (nh_.hasParam("reference_frame"))
	{
		nh_.getParam("reference_frame", referenceFrame_);
	}else{ success = false; }
	
	if (nh_.hasParam("target_frame"))
	{
		nh_.getParam("target_frame", targetFrame_);
	}else{ success = false; }
	
	if (nh_.hasParam("endeffector_frame"))
	{
		nh_.getParam("endeffector_frame", endeffectorFrame_);
	}else{ success = false; }
	
	
	stringPath_ = ros::package::getPath("cob_path_broadcaster"); 
	stringPath_ = stringPath_+"/movement/"+fileName_;	
	charPath_ = stringPath_.c_str();
	
	marker1_=0;
	marker2_=0;
	
	vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	speed_pub_ = nh_.advertise<std_msgs::Float64> ("linear_vel", 1);
	accl_pub_ = nh_.advertise<std_msgs::Float64> ("linear_accl", 1);
	path_pub_ = nh_.advertise<std_msgs::Float64> ("linear_path", 1);
	jerk_pub_ = nh_.advertise<std_msgs::Float64> ("linear_jerk", 1);
	
	ROS_WARN("Waiting for Services...");
	success = ros::service::waitForService("/arm_controller/start_tracking");
	success = ros::service::waitForService("/arm_controller/stop_tracking");
	startTracking_ = nh_.serviceClient<cob_srvs::SetString>("/arm_controller/start_tracking");
	stopTracking_ = nh_.serviceClient<std_srvs::Empty>("/arm_controller/stop_tracking");
	ROS_INFO("...done!");
	
	if(success){
		return true;
	}else{
		return false;
	}
}



void CobArticulation::load()
{	
	stop_tracking();
	ROS_INFO("Stopping current tracking");
	
	std::vector <geometry_msgs::Pose> posVec;
	geometry_msgs::Pose pose,actualTcpPose,start,end;
	tf::Quaternion q;
	tf::Transform trans;
	double roll_actual,pitch_actual,yaw_actual,roll,pitch,yaw,quat_x,quat_y,quat_z,quat_w;
	double x,y,z,x_new,y_new,z_new,x_center,y_center,z_center;
	double r,holdTime,vel,accl,startAngle,endAngle;
	std::string profile,rotateOnly;
	bool justRotate;
	
	TiXmlDocument doc(charPath_);
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
	{
		ROS_INFO("load okay");
		start_tracking();

		TiXmlHandle docHandle( &doc );
		TiXmlElement* child = docHandle.FirstChild( "Movement" ).FirstChild( "Move" ).ToElement();
			
		std::string movement;
		
		for( child; child; child=child->NextSiblingElement() )
		{		
			movement = child->Attribute( "move");

			if ("move_lin" == movement){
				ROS_INFO("move_linear");
				
				// Read Attributes
				x_new = atof(child->Attribute( "x"));
				y_new = atof(child->Attribute( "y"));
				z_new = atof(child->Attribute( "z"));
				roll = atof(child->Attribute( "roll"));
				pitch = atof(child->Attribute( "pitch"));
				yaw = atof(child->Attribute( "yaw"));
				vel = atof(child->Attribute( "vel"));
				accl = atof(child->Attribute( "accl"));
				profile = child->Attribute( "profile");
				rotateOnly = child->Attribute( "RotateOnly");
				
				roll*=M_PI/180;
				pitch*=M_PI/180;
				yaw*=M_PI/180;
				
				if(rotateOnly == "Yes")
					justRotate=true;
				else
					justRotate=false;

				
				// Transform RPY to Quaternion
				q.setRPY(roll,pitch,yaw);
				trans.setRotation(q);
				
				// Define End Pose
				end.position.x = x_new;
				end.position.y = y_new;
				end.position.z = z_new;
				end.orientation.x = trans.getRotation()[0];
				end.orientation.y = trans.getRotation()[1];
				end.orientation.z = trans.getRotation()[2];
				end.orientation.w = trans.getRotation()[3];
				
				PoseToRPY(actualTcpPose,roll,pitch,yaw);
				ROS_INFO("..........................actualTcpPose roll: %f pitch: %f yaw: %f",roll,pitch,yaw);
				// Interpolate the path
				linear_interpolation(&posVec,actualTcpPose,end,vel,accl,profile,justRotate);		
				
				// Broadcast the linearpath
				pose_path_broadcaster(&posVec);
				
				actualTcpPose=end;
				PoseToRPY(end,roll,pitch,yaw);
				ROS_INFO("..........................endpose roll: %f pitch: %f yaw: %f",roll,pitch,yaw);

			}
			
			if("move_ptp" == movement){
				ROS_INFO("move_ptp");
				x_new = atof(child->Attribute( "x"));
				y_new = atof(child->Attribute( "y"));
				z_new = atof(child->Attribute( "z"));
				roll = atof(child->Attribute( "roll"));
				pitch = atof(child->Attribute( "pitch"));
				yaw = atof(child->Attribute( "yaw"));
				
				roll*=M_PI/180;
				pitch*=M_PI/180;
				yaw*=M_PI/180;
				
				// Transform RPY to Quaternion
				q.setRPY(roll,pitch,yaw);
				trans.setRotation(q);
				
				pose.position.x = x_new;
				pose.position.y = y_new;
				pose.position.z = z_new;
				pose.orientation.x = trans.getRotation()[0];
				pose.orientation.y = trans.getRotation()[1];
				pose.orientation.z = trans.getRotation()[2];
				pose.orientation.w = trans.getRotation()[3];
				
				move_ptp(pose,0.03);
				
				actualTcpPose = pose;
				PoseToRPY(actualTcpPose,roll,pitch,yaw);
				ROS_INFO("PTP End Orientation: %f %f %f",roll,pitch,yaw);
			}
											
			if("move_circ" == movement){
				ROS_INFO("move_circ");
				
				x_center 	= atof(child->Attribute( "x_center"));
				y_center 	= atof(child->Attribute( "y_center"));
				z_center	= atof(child->Attribute( "z_center"));
				roll 		= atof(child->Attribute( "roll_center"));
				pitch		= atof(child->Attribute( "pitch_center"));
				yaw 		= atof(child->Attribute( "yaw_center"));
				r			= atof(child->Attribute( "r"));
				startAngle  = atof(child->Attribute( "startangle"));
				endAngle	= atof(child->Attribute( "endangle"));
				vel	  		= atof(child->Attribute( "vel"));
				accl 		= atof(child->Attribute( "accl"));
				profile 	= child->Attribute( "profile");


				pose = getEndeffectorPose();
				quat_x = pose.orientation.x;
				quat_y = pose.orientation.y;
				quat_z = pose.orientation.z;
				quat_w = pose.orientation.w;

				int marker3=0;
				
				showDot(x_center,y_center,z_center,marker3,1.0,0,0,"Center_point");
				circular_interpolation(&posVec,x_center,y_center,z_center,roll,pitch,yaw,startAngle,endAngle,r,vel,accl,profile);
				
				move_ptp(posVec.at(0),0.03);
				pose_path_broadcaster(&posVec);	
				actualTcpPose=posVec.at(posVec.size()-1);		
			}
		
			
			if("hold" == movement){		
				ROS_INFO("Hold position");
				holdTime = atof(child->Attribute( "time"));
				ros::Timer timer = nh_.createTimer(ros::Duration(holdTime), &CobArticulation::timerCallback, this);
				hold_=true;
				PoseToRPY(actualTcpPose,roll,pitch,yaw);
				ROS_INFO("Hold Orientation: %f %f %f",roll,pitch,yaw);
				hold_position(actualTcpPose);
			}
			posVec.clear();
		}	
	}else{
		ROS_WARN("Error loading File");
	}
	stop_tracking();
}

// Pseudo PTP
void CobArticulation::move_ptp(geometry_msgs::Pose targetPose, double epsilon){
	reached_pos_=false;
	int reached_pos_counter=0;	
	double ro,pi,ya;
	ros::Rate rate(update_rate_);
	ros::Time now;
	tf::StampedTransform stampedTransform;
	tf::Quaternion q;
	bool transformed=false;
	
	while(ros::ok()){
		now = ros::Time::now();

		// Linearkoordinaten
		transform_.setOrigin( tf::Vector3(targetPose.position.x,targetPose.position.y,targetPose.position.z) );
	
		q = tf::Quaternion(targetPose.orientation.x,targetPose.orientation.y,targetPose.orientation.z,targetPose.orientation.w);
		transform_.setRotation(q);

		// Send br Frame
		br_.sendTransform(tf::StampedTransform(transform_, now, referenceFrame_, targetFrame_));
		
		// Get transformation
		try{
		listener_.waitForTransform(targetFrame_,endeffectorFrame_, now, ros::Duration(0.5));
		listener_.lookupTransform(targetFrame_,endeffectorFrame_, now, stampedTransform);
		}
		catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		}
		
		// Get current RPY out of quaternion
		tf::Quaternion quatern = stampedTransform.getRotation();
		tf::Matrix3x3(quatern).getRPY(ro,pi,ya);
				
				
		// Wait for arm_7_link to be in position
		if(epsilon_area(stampedTransform.getOrigin().x(), stampedTransform.getOrigin().y(), stampedTransform.getOrigin().z(),ro,pi,ya,epsilon)){
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



void CobArticulation::pose_path_broadcaster(std::vector <geometry_msgs::Pose> *poseVector){
	ros::Rate rate(update_rate_);
	ros::Time now;
	tf::Quaternion q;
	
	double T_IPO=pow(update_rate_,-1);

	
	for(int i=0; i<poseVector->size()-1; i++){
		now = ros::Time::now();
		
	
		// Linearkoordinaten
		transform_.setOrigin(tf::Vector3(poseVector->at(i).position.x, poseVector->at(i).position.y, poseVector->at(i).position.z));
		
		q = tf::Quaternion(poseVector->at(i).orientation.x,poseVector->at(i).orientation.y,poseVector->at(i).orientation.z,poseVector->at(i).orientation.w);
		
		transform_.setRotation(q);   
		
		showMarker(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, targetFrame_),marker1_, 0 , 1.0 , 0 ,"goalFrame");
		//showMarker(stampedTransform_,marker2_, 1.0 , 0 , 0 , "TCP");				
		
		br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, targetFrame_));
		

		marker1_++;
		//marker2_++;
		
		ros::spinOnce();
		rate.sleep();
	}
	
}


void CobArticulation::hold_position(geometry_msgs::Pose holdPose)
{	
	ros::Rate rate(update_rate_);
	ros::Time now;
	tf::Quaternion q;
	
	while(hold_){
		now = ros::Time::now();
		
		// Linearcoordinates
		transform_.setOrigin( tf::Vector3(holdPose.position.x,holdPose.position.y,holdPose.position.z) );
	
		// RPY Angles
		q = tf::Quaternion(holdPose.orientation.x,holdPose.orientation.y,holdPose.orientation.z,holdPose.orientation.w);
		transform_.setRotation(q);   	
	
		br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, targetFrame_));
		ros::spinOnce();
		rate.sleep();
	}
}


// Helper Functions 
//--------------------------------------------------------------------------------------------------------------
void CobArticulation::linear_interpolation(	std::vector <geometry_msgs::Pose> *poseVector,
											geometry_msgs::Pose start, geometry_msgs::Pose end,
											double VelMax, double AcclMax, std::string profile,bool justRotate) 
{     
	geometry_msgs::Pose pose;
	tf::Quaternion q;
	tf::Transform transform;
	std::vector<double> pathMatrix[4];
	std::vector<double> linearPath,rollPath,pitchPath,yawPath;
	double start_roll, start_pitch, start_yaw;
	double end_roll, end_pitch, end_yaw;
	double Se = sqrt(pow((end.position.x-start.position.x),2)+pow((end.position.y-start.position.y),2)+pow((end.position.z-start.position.z),2));
	
	// Convert Quaternions into RPY Angles for start and end pose
	q = tf::Quaternion(start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w);
	tf::Matrix3x3(q).getRPY(start_roll,start_pitch,start_yaw);	
	q = tf::Quaternion(end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w);
	tf::Matrix3x3(q).getRPY(end_roll,end_pitch,end_yaw);	
	
	// Calculate path length for the angular movement
	double Se_roll,Se_pitch,Se_yaw;
	Se_roll = end_roll - start_roll;
	Se_pitch = end_pitch - start_pitch;
	Se_yaw = end_yaw - start_yaw;
	
	// Calculate path for each Angle
	calculateProfileForAngularMovements(pathMatrix,Se,Se_roll,Se_pitch,Se_yaw,start_roll,start_pitch,start_yaw,VelMax,AcclMax,profile,justRotate);
	
	linearPath=pathMatrix[0];
	rollPath=pathMatrix[1];
	pitchPath=pathMatrix[2];
	yawPath=pathMatrix[3];
	
	/*
	for(int i=0;i<linearPath.size();i++){
		ROS_INFO("linearPath[%i] = %f rollPath[%i] = %f  pitchPath[%i] = %f  yawPath[%i] = %f",i,linearPath.at(i),i,rollPath.at(i),i,pitchPath.at(i),i,yawPath.at(i));
	}
	*/
	
	// Interpolate the linear path
	for(int i=0;i<pathMatrix[0].size();i++){	
		if(!justRotate){
			pose.position.x = start.position.x + linearPath.at(i) * (end.position.x-start.position.x)/Se;
			pose.position.y = start.position.y + linearPath.at(i) * (end.position.y-start.position.y)/Se;
			pose.position.z = start.position.z + linearPath.at(i) * (end.position.z-start.position.z)/Se;	
		}
		else{
			pose.position.x = start.position.x;
			pose.position.y = start.position.y;
			pose.position.z = start.position.z;	
		}
		
		// Transform RPY to Quaternion
		q.setRPY(rollPath.at(i),pitchPath.at(i),yawPath.at(i));
		transform.setRotation(q);
	
		// Get Quaternion Values
		pose.orientation.x = transform.getRotation()[0];
		pose.orientation.y = transform.getRotation()[1];
		pose.orientation.z = transform.getRotation()[2];
		pose.orientation.w = transform.getRotation()[3];
		poseVector->push_back(pose);
	}	
}




void CobArticulation::circular_interpolation(	std::vector<geometry_msgs::Pose>* poseVector,
														double M_x,double M_y,double M_z,
														double M_roll,double M_pitch,double M_yaw,
														double startAngle, double endAngle,double r, double VelMax, double AcclMax,
														std::string profile)  
{
	tf::Transform C,P,T;
	tf::Quaternion q;
	geometry_msgs::Pose pose,pos;
	std::vector<double> pathArray;
	
	
	// Convert RPY angles into [RAD]
	startAngle=startAngle*M_PI/180;
	endAngle=endAngle*M_PI/180;
	M_roll = M_roll*M_PI/180;
	M_pitch = M_pitch*M_PI/180;
	M_yaw = M_yaw*M_PI/180;
	
	double Se = endAngle-startAngle;
	bool forward;
	
	if(Se < 0)
		forward=false;
	else
		forward=true;
		
	Se = std::fabs(Se);
	
	
	// Calculates the Path with Ramp - or Sinoidprofile
	calculateProfile(&pathArray,Se,VelMax,AcclMax,profile);
		
	// Define Center Pose
	C.setOrigin( tf::Vector3(M_x,M_y,M_z) );
	q.setRPY(M_roll,M_pitch,M_yaw);
    C.setRotation(q);

	// Interpolate the circular path
	for(int i=0;i<pathArray.size();i++){
		// Rotate T
		T.setOrigin(tf::Vector3(cos(pathArray.at(i))*r,0,sin(pathArray.at(i))*r));
		
		if(endAngle<startAngle){
				T.setOrigin(tf::Vector3(cos(pathArray.at(i))*r,0,sin(pathArray.at(i))*r));
				q.setRPY(0,-pathArray.at(i),0);			
		}else{
				T.setOrigin(tf::Vector3(cos(startAngle-pathArray.at(i))*r,0,sin(startAngle-pathArray.at(i))*r));
				q.setRPY(0,pathArray.at(i),0);
		}	
		
		T.setRotation(q);	
			
		// Calculate TCP Position
		P = C * T;
		
		// Fill the Pose
		pose.position.x = P.getOrigin().x();
		pose.position.y = P.getOrigin().y();
		pose.position.z = P.getOrigin().z();
		
		pose.orientation.x = P.getRotation()[0];
		pose.orientation.y = P.getRotation()[1];
		pose.orientation.z = P.getRotation()[2];
		pose.orientation.w = P.getRotation()[3];	
		
		// Put the pose into the pose Vector
		poseVector->push_back(pose);
		
	}
	
	// Visualize center point 
	int marker4=0;
	q.setRPY(M_roll+(M_PI/2),M_pitch,M_yaw);
    C.setRotation(q);
	showLevel(C,marker4,1.0,0,0,"level");
}


void CobArticulation::calculateProfile(std::vector<double> *pathArray,double Se, double VelMax, double AcclMax, std::string profile)
{	
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


void CobArticulation::calculateProfileForAngularMovements(std::vector<double> *pathMatrix,
										 				 double Se, double Se_roll, double Se_pitch, double Se_yaw,
										 				 double start_angle_roll, double start_angle_pitch, double start_angle_yaw,
														 double VelMax, double AcclMax,std::string profile, bool justRotate)
{	
	std::vector<double> linearPath,rollPath,pitchPath,yawPath;
	int steps_te,steps_tv,steps_tb=0;
	double tv,tb,te=0;
	double T_IPO=pow(update_rate_,-1);
	double params[4][2];
	double Se_max,temp=0;

	double Se_array[4] = {Se,Se_roll,Se_pitch,Se_yaw};
	
	for(int i=0;i<sizeof(Se_array);i++){
		if(temp<std::fabs(Se_array[i]))
			temp=std::fabs(Se_array[i]);
	}
	
	// If justRoate == true, then set the largest angular difference as Se_max.
	if(justRotate){
		Se_max=temp;
	}
	else{	// Otherwise set the linear-path as Se_max
		Se_max = Se;
	}
	
	
	// Calculate the Profile Timings for the linear-path
	if(profile == "ramp"){
		// Calculate the Ramp Profile Parameters
		if (VelMax > sqrt(std::fabs(Se_max)*AcclMax)){
			VelMax = sqrt(std::fabs(Se_max)*AcclMax);
		}
		tb = VelMax/AcclMax;
		te = (std::fabs(Se_max) / VelMax) + tb;
		tv = te - tb;
	}
	else{
		// Calculate the Sinoide Profile Parameters
		if (VelMax > sqrt(std::fabs(Se_max)*AcclMax/2)){
			VelMax = sqrt(std::fabs(Se_max)*AcclMax/2);
		}
		tb = 2*VelMax/AcclMax;
		te = (std::fabs(Se_max) / VelMax) + tb;
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
		
	// Calculate the paths
	generatePath(&linearPath,	T_IPO,VelMax,AcclMax,Se_max,(steps_tb+steps_tv+steps_te),profile);
	generatePathWithTe(&rollPath, T_IPO, te, AcclMax, Se_roll, (steps_tb+steps_tv+steps_te),start_angle_roll,profile);
	generatePathWithTe(&pitchPath, T_IPO, te, AcclMax, Se_pitch, (steps_tb+steps_tv+steps_te),start_angle_pitch,profile);
	generatePathWithTe(&yawPath, T_IPO, te, AcclMax, Se_yaw, (steps_tb+steps_tv+steps_te),start_angle_yaw,profile);
	

	// Get the Vecotr sizes of each path-vector
	int MaxStepArray[4],maxSteps;
	MaxStepArray[0] = linearPath.size();
	MaxStepArray[1] = rollPath.size();
	MaxStepArray[2] = pitchPath.size();
	MaxStepArray[3] = yawPath.size();
	
	// Get the largest one
	maxSteps = 0;
	for(int i=0;i<4;i++){
		if(maxSteps<MaxStepArray[i])
			maxSteps=MaxStepArray[i];
	}
	
	
	bool linearOkay,rollOkay,pitchOkay,yawOkay=false;
	
	// Check if every vector has the same length than the largest one.
	while(true){
		if(linearPath.size() < maxSteps){
		linearPath.push_back(linearPath.at(linearPath.size()-1));	
 		}else{linearOkay=true;}
		
		if(rollPath.size() < maxSteps){
			rollPath.push_back(rollPath.at(rollPath.size()-1));
		}else{rollOkay=true;}
		
		if(pitchPath.size() < maxSteps){
			pitchPath.push_back(pitchPath.at(pitchPath.size()-1));
		}else{pitchOkay=true;}
		
		if(yawPath.size() < maxSteps){
			yawPath.push_back(yawPath.at(yawPath.size()-1));
		}else{yawOkay=true;}
		
		if(linearOkay && rollOkay && pitchOkay && yawOkay){
			break;
		}	
	}

	
	// Put the interpolated paths into the pathMatrix
	pathMatrix[0]=linearPath;
	pathMatrix[1]=rollPath;
	pathMatrix[2]=pitchPath;
	pathMatrix[3]=yawPath;
}





geometry_msgs::Pose CobArticulation::getEndeffectorPose()
{	
	geometry_msgs::Pose pos;	
	tf::StampedTransform stampedTransform;
	bool transformed=false;
	do{
		// Get transformation
		try{
			listener_.lookupTransform(referenceFrame_,endeffectorFrame_, ros::Time(0), stampedTransform);
			transformed=true;
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			transformed = false;
			ros::Duration(0.1).sleep();
		}	
	}while(!transformed);
	
	currentEndeffectorStampedTransform_ = stampedTransform;
	
	pos.position.x=stampedTransform.getOrigin().x();
	pos.position.y=stampedTransform.getOrigin().y();
	pos.position.z=stampedTransform.getOrigin().z();
	pos.orientation.x = stampedTransform.getRotation()[0];
	pos.orientation.y = stampedTransform.getRotation()[1];
	pos.orientation.z = stampedTransform.getRotation()[2];
	pos.orientation.w = stampedTransform.getRotation()[3];
			
	return pos;
}



// Checks if the endeffector is in the area of the 'br' frame
bool CobArticulation::epsilon_area(double x,double y, double z, double roll, double pitch, double yaw,double epsilon)
{
	bool x_okay=false, y_okay=false, z_okay=false;
	bool roll_okay=false, pitch_okay=false, yaw_okay=false;
	
	x=std::fabs(x);
	y=std::fabs(y);
	z=std::fabs(z);
	roll=std::fabs(roll);
	pitch=std::fabs(pitch);
	yaw=std::fabs(yaw);
	
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



void CobArticulation::showMarker(tf::StampedTransform tf,int marker_id,double red, double green, double blue,std::string ns)
{
  	visualization_msgs::Marker marker;
	marker.header.frame_id = referenceFrame_;
	marker.header.stamp = ros::Time();
	marker.ns = ns;
	marker.id = marker_id;
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
	marker.color.r = red;
	marker.color.g = green;
	marker.color.b = blue;
	
	marker.color.a = 1.0;

	vis_pub_.publish( marker );
	
}

void CobArticulation::showDot(double x,double y,double z,int marker_id,double red, double green, double blue,std::string ns)
{
  	visualization_msgs::Marker marker;
	marker.header.frame_id = referenceFrame_;
	marker.header.stamp = ros::Time();
	marker.ns = ns;
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
		
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.r = red;
	marker.color.g = green;
	marker.color.b = blue;
	
	marker.color.a = 1.0;

	vis_pub_.publish( marker );	
}

void CobArticulation::showLevel(tf::Transform pos,int marker_id,double red, double green, double blue,std::string ns)
{
  	visualization_msgs::Marker marker;
	marker.header.frame_id = referenceFrame_;
	marker.header.stamp = ros::Time();
	marker.ns = ns;
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
		
	marker.pose.position.x = pos.getOrigin().x();
	marker.pose.position.y = pos.getOrigin().y();
	marker.pose.position.z = pos.getOrigin().z();
	marker.pose.orientation.x = pos.getRotation()[0];
	marker.pose.orientation.y = pos.getRotation()[1];
	marker.pose.orientation.z = pos.getRotation()[2];
	marker.pose.orientation.w = pos.getRotation()[3];
	
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 0.01;
	marker.color.r = red;
	marker.color.g = green;
	marker.color.b = blue;
	
	marker.color.a = 0.2;

	vis_pub_.publish( marker );	
}


void CobArticulation::timerCallback(const ros::TimerEvent& event){
	hold_=false;
}


void CobArticulation::start_tracking()
{	
    cob_srvs::SetString start;
    start.request.data = targetFrame_;
    startTracking_.call(start);
    
    if(start.response.success==true){
		ROS_INFO("...service called!");
	}
	else{
		ROS_INFO("...service failed");
	}
}


void CobArticulation::stop_tracking()
{
    std_srvs::Empty srv_save_stop;
    srv_save_stop.request;
    if (stopTracking_.call(srv_save_stop)) {
		ROS_INFO("... service stopped!");
		}
    	else {
			ROS_ERROR("... service stop failed! FATAL!");
		}
}

void CobArticulation::generatePath(std::vector<double> *pathArray,double T_IPO, double VelMax, double AcclMax,double Se_max, int steps_max, std::string profile){	
	double tv,tb,te=0;
	int steps_te,steps_tv,steps_tb=0;
	
	// Reconfigure the timings and parameters with T_IPO
	tb = (VelMax/(AcclMax*T_IPO)) * T_IPO;
	tv = (std::fabs(Se_max)/(VelMax*T_IPO)) * T_IPO;
	te=tv+tb;
	VelMax = std::fabs(Se_max) / tv;
	AcclMax = VelMax / tb;
	
	// Calculate the Profile Timings for the longest path
	if(profile == "ramp"){
		tb = VelMax/AcclMax;
		te = (std::fabs(Se_max) / VelMax) + tb;
		tv = te - tb;
	}
	else{
		tb = 2*VelMax/AcclMax;
		te = (std::fabs(Se_max) / VelMax) + tb;
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
	
	if(profile == "ramp"){
		// Calculate the ramp profile path
		// 0 <= t <= tb
		for(int i=0;i<=steps_tb-1;i++){	
			pathArray->push_back( Se_max/std::fabs(Se_max)*(0.5*AcclMax*pow((T_IPO*i),2)));
		}
		// tb <= t <= tv
		for(int i=steps_tb;i<=(steps_tb+steps_tv-1);i++){	
			pathArray->push_back(Se_max/std::fabs(Se_max)*(VelMax*(T_IPO*i)-0.5*pow(VelMax,2)/AcclMax));
		}
		// tv <= t <= te
		for(int i=(steps_tb+steps_tv);i<(steps_tv+steps_tb+steps_te-1);i++){	
			pathArray->push_back(Se_max/std::fabs(Se_max)*(VelMax * (te-tb) - 0.5*AcclMax* pow(te-(i*T_IPO),2)));
		}
	}
	else{
		// Calculate the sinoide profile path
		// 0 <= t <= tb
		for(int i=0;i<=steps_tb-1;i++){	
			pathArray->push_back( Se_max/std::fabs(Se_max)*( AcclMax*(0.25*pow(i*T_IPO,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*T_IPO))-1))  ));
		}
		// tb <= t <= tv
		for(int i=steps_tb;i<=(steps_tb+steps_tv-1);i++){	
			pathArray->push_back(Se_max/std::fabs(Se_max)*(VelMax*(i*T_IPO-0.5*tb)));
		}
		// tv <= t <= te
		for(int i=(steps_tb+steps_tv);i<(steps_tv+steps_tb+steps_te-1);i++){	
			pathArray->push_back(Se_max/std::fabs(Se_max)*(0.5*AcclMax*( te*(i*T_IPO + tb)  -  0.5*(pow(i*T_IPO,2)+pow(te,2)+2*pow(tb,2))  + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos( ((2*M_PI)/tb) * (i*T_IPO-tv))))));
		}
	}
}


void CobArticulation::generatePathWithTe(std::vector<double> *pathArray,double T_IPO, double te, double AcclMax,double Se_max, int steps_max,double start_angle,std::string profile){	
	double tv,tb=0;
	int steps_te,steps_tv,steps_tb=0;
	double VelMax;								

	
	// Calculate the Profile Timings
	if(profile == "ramp"){
		// Reconfigure AcclMax and Velmax
		while(te< 2*sqrt(std::fabs(Se_max)/AcclMax)){
			AcclMax+=0.001;
		}
		VelMax = AcclMax * te / 2 - sqrt((pow(AcclMax,2)*pow(te,2)/4) - std::fabs(Se_max) * AcclMax );
		
		// Calculate profile timings, te is known
		tb = VelMax/AcclMax;
		tv = te - tb;
	}
	else{	// Sinoide
		// Reconfigure AcclMax and Velmax
		while(te< sqrt(std::fabs(Se_max)*8/AcclMax)){
			AcclMax+=0.001;
		}
		VelMax = AcclMax * te / 4 - sqrt((pow(AcclMax,2)*pow(te,2)/16) - std::fabs(Se_max) * AcclMax/2 );
		
		// Calculate profile timings, te is known
		tb = 2*VelMax/AcclMax;
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
	

	if(profile == "ramp"){
		// Calculate the ramp profile path
		// 0 <= t <= tb
		for(int i=0;i<=steps_tb-1;i++){	
			pathArray->push_back( start_angle + Se_max/std::fabs(Se_max)*(0.5*AcclMax*pow((T_IPO*i),2)));
		}
		// tb <= t <= tv
		for(int i=steps_tb;i<=(steps_tb+steps_tv-1);i++){	
			pathArray->push_back(start_angle + Se_max/std::fabs(Se_max)*(VelMax*(T_IPO*i)-0.5*pow(VelMax,2)/AcclMax));
		}
		// tv <= t <= te
		for(int i=(steps_tb+steps_tv);i<(steps_tv+steps_tb+steps_te-1);i++){	
			pathArray->push_back(start_angle + Se_max/std::fabs(Se_max)*(VelMax * (te-tb) - 0.5*AcclMax* pow(te-(i*T_IPO),2)));
		}
	}
	else{
		// Calculate the sinoide profile path
		// 0 <= t <= tb
		for(int i=0;i<=steps_tb-1;i++){	
			pathArray->push_back(start_angle + Se_max/std::fabs(Se_max)*( AcclMax*(0.25*pow(i*T_IPO,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*T_IPO))-1))  ));
		}
		// tb <= t <= tv
		for(int i=steps_tb;i<=(steps_tb+steps_tv-1);i++){	
			pathArray->push_back(start_angle + Se_max/std::fabs(Se_max)*(VelMax*(i*T_IPO-0.5*tb)));
		}
		// tv <= t <= te
		for(int i=(steps_tb+steps_tv);i<(steps_tv+steps_tb+steps_te-1);i++){	
			pathArray->push_back(start_angle + Se_max/std::fabs(Se_max)*(0.5*AcclMax*( te*(i*T_IPO + tb)  -  0.5*(pow(i*T_IPO,2)+pow(te,2)+2*pow(tb,2))  + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos( ((2*M_PI)/tb) * (i*T_IPO-tv))))));
		}
	}
}


void CobArticulation::PoseToRPY(geometry_msgs::Pose pose,double &roll, double &pitch, double &yaw){
	tf::Quaternion q = tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
	tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
}
