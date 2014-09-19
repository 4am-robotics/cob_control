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
	
	marker1_=0;
	marker2_=0;
	start_tracking();
	
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
	stop_tracking();
}



void CobArticulation::load(const char* pFilename)
{	

	std::vector <geometry_msgs::Pose> posVec;
	geometry_msgs::Pose pose,p,actualTcpPosition;
	tf::Quaternion q;
	geometry_msgs::Pose start,end;
	tf::Transform trans;
	double roll_actual,pitch_actual,yaw_actual;
	
	TiXmlDocument doc(pFilename);
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
	{
		ROS_INFO("load okay");

		TiXmlHandle docHandle( &doc );
		TiXmlElement* child = docHandle.FirstChild( "Movement" ).FirstChild( "Move" ).ToElement();
		

		
		std::string movement;
		
		for( child; child; child=child->NextSiblingElement() )
		{		
			movement = child->Attribute( "move");

			if ("move_lin" == movement){

				bool justRotate;
				
				ROS_INFO("move_linear");
				
				// Read Attributes
				x_new_ = atof(child->Attribute( "x"));
				y_new_ = atof(child->Attribute( "y"));
				z_new_ = atof(child->Attribute( "z"));
				roll_ = atof(child->Attribute( "roll"));
				pitch_ = atof(child->Attribute( "pitch"));
				yaw_ = atof(child->Attribute( "yaw"));
				vel_ = atof(child->Attribute( "vel"));
				accl_ = atof(child->Attribute( "accl"));
				profile_ = child->Attribute( "profile");
				justRotate_ = child->Attribute( "RotateOnly");
				
				roll_=roll_*M_PI/180;
				pitch_=pitch_*M_PI/180;
				yaw_=yaw_*M_PI/180;
				
				if(justRotate_ == "Yes")
					justRotate=true;
				else
					justRotate=false;
				
				// Get actuall position and orientation
				p = getEndeffectorPosition();
				x_=p.position.x;
				y_=p.position.y;
				z_=p.position.z;
				quat_x_ = p.orientation.x;
				quat_y_ = p.orientation.y;
				quat_z_ = p.orientation.z;
				quat_w_ = p.orientation.w;

				// Define Start Pose
				start.position.x = x_;
				start.position.y = y_;
				start.position.z = z_;
				start.orientation.x = quat_x_;
				start.orientation.y = quat_y_;
				start.orientation.z = quat_z_;
				start.orientation.w = quat_w_;
				
				
				//start = actualTcpPosition;
				
				// Transform RPY to Quaternion
				q.setRPY(roll_,pitch_,yaw_);
				trans.setRotation(q);
				
				// Define End Pose
				end.position.x = x_new_;
				end.position.y = y_new_;
				end.position.z = z_new_;
				end.orientation.x = trans.getRotation()[0];
				end.orientation.y = trans.getRotation()[1];
				end.orientation.z = trans.getRotation()[2];
				end.orientation.w = trans.getRotation()[3];
				
				q = tf::Quaternion(trans.getRotation()[0],trans.getRotation()[1],trans.getRotation()[2],trans.getRotation()[3]);
				tf::Matrix3x3(q).getRPY(roll_,pitch_,yaw_);	
				ROS_INFO("CALL ...   Quaternions: %f %f %f %f",trans.getRotation()[0],trans.getRotation()[1],trans.getRotation()[2],trans.getRotation()[3]);
				ROS_INFO("Call ...   roll: %f   pitch: %f   yaw: %f",roll_,pitch_,yaw_);
				
				// Interpolate the path
				linear_interpolation(&posVec,start,end,vel_,accl_,profile_,justRotate);		
				
				// Broadcast the linearpath
				pose_path_broadcaster(&posVec);
				
				actualTcpPosition=end;
			}
			
			if("move_ptp" == movement){
				ROS_INFO("move_ptp");
				x_new_ = atof(child->Attribute( "x"));
				y_new_ = atof(child->Attribute( "y"));
				z_new_ = atof(child->Attribute( "z"));
				roll_ = atof(child->Attribute( "roll"));
				pitch_ = atof(child->Attribute( "pitch"));
				yaw_ = atof(child->Attribute( "yaw"));
				
				roll_=roll_*M_PI/180;
				pitch_=pitch_*M_PI/180;
				yaw_=yaw_*M_PI/180;
				
				move_ptp(x_new_,y_new_,z_new_,roll_,pitch_,yaw_,0.03);
				
				actualTcpPosition.position.x = x_new_;
				actualTcpPosition.position.y = y_new_;
				actualTcpPosition.position.z = z_new_;
				
				// Transform RPY to Quaternion
				q.setRPY(roll_,pitch_,yaw_);
				trans.setRotation(q);
				
				actualTcpPosition.orientation.x = trans.getRotation()[0];
				actualTcpPosition.orientation.y = trans.getRotation()[1];
				actualTcpPosition.orientation.z = trans.getRotation()[2];
				actualTcpPosition.orientation.w = trans.getRotation()[3];
			}
											
			if("move_circ_any_level" == movement){
				ROS_INFO("move_circ");
				
				x_center_ 	= atof(child->Attribute( "x_center"));
				y_center_ 	= atof(child->Attribute( "y_center"));
				z_center_	= atof(child->Attribute( "z_center"));
				roll_ 		= atof(child->Attribute( "roll_center"));
				pitch_		= atof(child->Attribute( "pitch_center"));
				yaw_ 		= atof(child->Attribute( "yaw_center"));
				r_			= atof(child->Attribute( "r"));
				startAngle_ = atof(child->Attribute( "startangle"));
				endAngle_	= atof(child->Attribute( "endangle"));
				vel_		= atof(child->Attribute( "vel"));
				accl_ 		= atof(child->Attribute( "accl"));
				profile_ 	= child->Attribute( "profile");


				p = getEndeffectorPosition();
				quat_x_ = p.orientation.x;
				quat_y_ = p.orientation.y;
				quat_z_ = p.orientation.z;
				quat_w_ = p.orientation.w;

				int marker3=0;
				
				showDot(x_center_,y_center_,z_center_,marker3,1.0,0,0,"Center_point");
				circular_interpolation_any_level(&posVec,x_center_,y_center_,z_center_,roll_,pitch_,yaw_,startAngle_,endAngle_,r_,vel_,accl_,profile_);
				pose_path_broadcaster(&posVec);
				
				
			}
		
			
			if("hold" == movement){
				geometry_msgs::Pose holdPose;
				// Get actuall position and orientation
				
				p = getEndeffectorPosition();
				holdPose.position.x=x_new_;
				holdPose.position.y=y_new_;
				holdPose.position.z=z_new_;
				holdPose.orientation.x = p.orientation.x;
				holdPose.orientation.y = p.orientation.y;
				holdPose.orientation.z = p.orientation.z;
				holdPose.orientation.w = p.orientation.w;
			
				
				ROS_INFO("Hold position");
				holdTime_ = atof(child->Attribute( "time"));
				ros::Timer timer = nh_.createTimer(ros::Duration(holdTime_), &CobArticulation::timerCallback, this);
				hold_=true;
				hold_position(actualTcpPosition);
			}

			x_=x_new_;
			y_=y_new_;
			z_=z_new_;
			posVec.clear();
		}	
	}else{
		ROS_WARN("Error loading File");
	}
}

// Pseudo PTP
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
		
		marker(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, goalFrame_),marker1_, 0 , 1.0 , 0 ,"goalFrame");
		//marker(stampedTransform_,marker2_, 1.0 , 0 , 0 , "TCP");				
		
		br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, goalFrame_));
		

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
		
		// Linearkoordinaten
		transform_.setOrigin( tf::Vector3(holdPose.position.x,holdPose.position.y,holdPose.position.z) );
	
		// RPY Winkel
		q = tf::Quaternion(holdPose.orientation.x,holdPose.orientation.y,holdPose.orientation.z,holdPose.orientation.w);
		transform_.setRotation(q);   	
	
		br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, goalFrame_));
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

	ROS_INFO("BLABLA   endroll: %f",end_roll);
	
	// Calculate path length for the angular movement
	double Se_roll = betrag(end_roll-start_roll);
	double Se_pitch = betrag(end_pitch-start_pitch);
	double Se_yaw = betrag(end_yaw-start_yaw);
	
	// Calculate path for each Angle
	calculateProfileForAngularMovements(pathMatrix,Se,Se_roll,Se_pitch,Se_yaw,VelMax,AcclMax,profile,justRotate);
	
	linearPath=pathMatrix[0];
	rollPath=pathMatrix[1];
	pitchPath=pathMatrix[2];
	yawPath=pathMatrix[3];
	
	for(int i=0;i<rollPath.size();i++){
		ROS_INFO("rollPath[%i] = %f",i,rollPath.at(i));
	}
	
	
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




void CobArticulation::circular_interpolation_any_level(	std::vector<geometry_msgs::Pose>* poseVector,
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
		
	Se = betrag(Se);
	
	
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
		if(temp<Se_array[i])
			temp=Se_array[i];
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
		if (VelMax > sqrt(Se_max*AcclMax)){
			VelMax = sqrt(Se_max*AcclMax);
		}
		tb = VelMax/AcclMax;
		te = (Se_max / VelMax) + tb;
		tv = te - tb;
	}
	else{
		// Calculate the Sinoide Profile Parameters
		if (VelMax > sqrt(Se_max*AcclMax/2)){
			VelMax = sqrt(Se_max*AcclMax/2);
		}
		tb = 2*VelMax/AcclMax;
		te = (Se_max / VelMax) + tb;
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
	
	// Constraint for linear path !!!
	// These parameters set the linear velocity and acceleration
	params[0][0] = VelMax;
	params[0][1] = AcclMax;
	
	
	// Calculate the velocity and acceleration of each angle in dependence of the linear path timings tb and tv.
	for(int i=1;i<4;i++){
		if(profile == "ramp"){
			params[i][1] = Se_array[i]		/ (tb*tv); 	// Acceleration
			params[i][0] = params[i][1] 	* tb;		// Velocity
		}
		else{
			params[i][1] = 2*Se_array[i]	/ (tb*tv); 	// Acceleration
			params[i][0] = params[i][1] 	* tb;		// Velocity
		}
	}
	
	for(int i=0;i<4;i++){
		ROS_INFO("params[%i][0] = %f",i,params[i][0]); 
		ROS_INFO("params[%i][1] = %f",i,params[i][1]); 
	}
	
	// Interpolate the paths
	generatePath(&linearPath,	T_IPO,params[0][0],params[0][1],Se_max,(steps_tb+steps_tv+steps_te),profile);
	generatePath(&rollPath,		T_IPO,params[1][0],params[1][1],Se_roll,(steps_tb+steps_tv+steps_te),profile);
	generatePath(&pitchPath,	T_IPO,params[2][0],params[2][1],Se_pitch,(steps_tb+steps_tv+steps_te),profile);
	generatePath(&yawPath,		T_IPO,params[3][0],params[3][1],Se_yaw,(steps_tb+steps_tv+steps_te),profile);

	// Put the interpolated paths into the pathMatrix
	pathMatrix[0]=linearPath;
	pathMatrix[1]=rollPath;
	pathMatrix[2]=pitchPath;
	pathMatrix[3]=yawPath;
}



double CobArticulation::betrag(double num){
	if(num<0){
		num=num*(-1.0);
	}
	return num;
}



geometry_msgs::Pose CobArticulation::getEndeffectorPosition()
{	
	geometry_msgs::Pose pos;	
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
	
	pos.position.x=stampedTransform_.getOrigin().x();
	pos.position.y=stampedTransform_.getOrigin().y();
	pos.position.z=stampedTransform_.getOrigin().z();
	pos.orientation.x = stampedTransform_.getRotation()[0];
	pos.orientation.y = stampedTransform_.getRotation()[1];
	pos.orientation.z = stampedTransform_.getRotation()[2];
	pos.orientation.w = stampedTransform_.getRotation()[3];
			
	return pos;
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



void CobArticulation::marker(tf::StampedTransform tf,int marker_id,double red, double green, double blue,std::string ns)
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
	ros::ServiceClient startTracking = nh_.serviceClient<cob_srvs::SetString>("/arm_controller/start_tracking");
    cob_srvs::SetString start;
    start.request.data = goalFrame_;
    startTracking.call(start);
    
    if(start.response.success==true){
		ROS_INFO("...service called!");
	}
	else{
		ROS_INFO("...service failed");
	}

}




void CobArticulation::stop_tracking()
{
	ros::ServiceClient savety_client = nh_.serviceClient<std_srvs::Empty>("/arm_controller/stop_tracking");
    std_srvs::Empty srv_save_stop;
    srv_save_stop.request;
    if (savety_client.call(srv_save_stop)) {
		ROS_INFO("Lookat stopped for savety issues");
		}
    	else {
			ROS_ERROR("Lookat stop failed! FATAL!");
		}
}

void CobArticulation::generatePath(std::vector<double> *pathArray,double T_IPO, double VelMax, double AcclMax,double Se_max, int steps_max, std::string profile){	
	double tv,tb,te=0;
	int steps_te,steps_tv,steps_tb=0;
	
	// Calculate the Profile Timings for the longest path
	if(profile == "ramp"){
		// Calculate the Ramp Profile Parameters
		if (VelMax > sqrt(Se_max*AcclMax)){
			VelMax = sqrt(Se_max*AcclMax);
		}
		tb = VelMax/AcclMax;
		te = (Se_max / VelMax) + tb;
		tv = te - tb;
	}
	else{
		// Calculate the Sinoide Profile Parameters
		if (VelMax > sqrt(Se_max*AcclMax/2)){
			VelMax = sqrt(Se_max*AcclMax/2);
		}
		tb = 2*VelMax/AcclMax;
		te = (Se_max / VelMax) + tb;
		tv = te - tb;		
	}
	
	
	// Interpolationsteps for every timesequence
	steps_tb = (double)tb / T_IPO;
	steps_tv = (double)(tv-tb) / T_IPO;
	steps_te = (double)(te-tv) / T_IPO;
	
	// Reconfigure the steps with the max step size
	while(true){
		if(steps_tb+steps_tv+steps_te < steps_max){
			steps_tv++;	
		}
		if(steps_tb+steps_tv+steps_te > steps_max){
			steps_tv--;
		}
		if(steps_tb+steps_tv+steps_te == steps_max){
			break;
		}
	}
	
	// Reconfigure timings wtih T_IPO
	tb=steps_tb*T_IPO;
	tv=(steps_tb+steps_tv)*T_IPO;
	te=(steps_tb+steps_tv+steps_te)*T_IPO;
	
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
			pathArray->push_back(VelMax * (te-tb) - 0.5*AcclMax* pow(te-(i*T_IPO),2));
		}
	}
	else{
		ROS_INFO("Sinoide Profile!?");
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

