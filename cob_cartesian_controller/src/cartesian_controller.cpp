/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
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
 *   ROS package name: cob_cartesian_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: July, 2015
 *
 * \brief
 *   ...
 *
 ****************************************************************/


#include <math.h>
#include <algorithm>

#include <ros/ros.h>
#include "ros/package.h"

#include <std_srvs/Empty.h>
#include <cob_srvs/SetString.h>

#include <kdl_conversions/kdl_msg.h>
#include <cob_cartesian_controller/cartesian_controller.h>
#include <cob_cartesian_controller/trajectory_interpolator/trajectory_interpolator.h>


bool CartesianController::initialize()
{
	ros::NodeHandle nh_private("~");
	
	///get params articulation Nodehandle
	if(!nh_private.getParam("file_name", fileName_))
	{
		ROS_ERROR("Parameter 'file_name' not set");
		return false;
	}
	
	if(!nh_private.getParam("reference_frame", referenceFrame_))
	{
		ROS_ERROR("Parameter 'reference_frame' not set");
		return false;
	}
	
	if(!nh_private.getParam("target_frame", targetFrame_))
	{
		ROS_ERROR("Parameter 'target_frame' not set");
		return false;
	}
	
	if (nh_private.hasParam("update_rate"))
	{	nh_private.getParam("update_rate", update_rate_);	}
	else
	{	update_rate_ = 68.0;	}	//hz


	/// Cartesian Nodehandle
	if (!nh_.getParam("chain_tip_link", chain_tip_link_))
	{
		ROS_ERROR("Parameter 'chain_tip_link' not set");
		return false;
	}
	

	stringPath_ = ros::package::getPath("cob_cartesian_controller"); 
	stringPath_ = stringPath_+"/movement/"+fileName_;
	charPath_ = stringPath_.c_str();
	
	marker1_=0;
	
	vis_pub_ = nh_private.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	speed_pub_ = nh_private.advertise<std_msgs::Float64> ("debug/linear_vel", 1);
	accl_pub_ = nh_private.advertise<std_msgs::Float64> ("debug/linear_accl", 1);
	path_pub_ = nh_private.advertise<std_msgs::Float64> ("debug/linear_path", 1);
	jerk_pub_ = nh_private.advertise<std_msgs::Float64> ("debug/linear_jerk", 1);
	
	ROS_WARN("Waiting for Services...");
	startTracking_ = nh_.serviceClient<cob_srvs::SetString>("frame_tracker/start_tracking");
	stopTracking_ = nh_.serviceClient<std_srvs::Empty>("frame_tracker/stop_tracking");
	startTracking_.waitForExistence();
	stopTracking_.waitForExistence();
	ROS_INFO("...done!");
	

	return true;
}

void CartesianController::load()
{
	stop_tracking();
	ROS_INFO("Stopping current tracking");
	std::vector <geometry_msgs::Pose> posVec;
	geometry_msgs::Pose pose,actualTcpPose,start,end;
	tf::Quaternion q,q_start,q_end, q_rel;
	tf::Transform trans,relative_diff;
	double roll_actual,pitch_actual,yaw_actual,roll,pitch,yaw,quat_x,quat_y,quat_z,quat_w;
	double x,y,z,x_new,y_new,z_new,x_center,y_center,z_center;
	double r,holdTime,vel,accl,startAngle,endAngle;
	std::string profile,rotateOnly;
	bool justRotate;
	
    TrajectoryInterpolator TIP(update_rate_);


	TiXmlDocument doc(charPath_);
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
	{
		ROS_INFO("load okay");
		start_tracking();
		
		TiXmlHandle docHandle( &doc );
		TiXmlElement* child = docHandle.FirstChild( "Movement" ).FirstChild( "Move" ).ToElement();
			
		std::string movement;
		
		for( child; child; child=child->NextSiblingElement())
		{
			movement = child->Attribute( "move");
			
			if ("move_lin" == movement){	// Relative position to endeffector ! 
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
				
				actualTcpPose = getEndeffectorPose();
				
				// Transform RPY to Quaternion
				q_rel.setRPY(roll,pitch,yaw);
				
				q_start = tf::Quaternion(actualTcpPose.orientation.x,
				                         actualTcpPose.orientation.y,
				                         actualTcpPose.orientation.z,
				                         actualTcpPose.orientation.w);

				q_end = q_start * q_rel;

				// Define End Pose
				end.position.x = actualTcpPose.position.x + x_new;
				end.position.y = actualTcpPose.position.y + y_new;
				end.position.z = actualTcpPose.position.z + z_new;
	            end.orientation.x = q_end.getX();
                end.orientation.y = q_end.getY();
                end.orientation.z = q_end.getZ();
                end.orientation.w = q_end.getW();

				actualTcpPose = getEndeffectorPose();
				PoseToRPY(actualTcpPose,roll,pitch,yaw);

				// Interpolate the path
				TIP.linear_interpolation(posVec,actualTcpPose,end,vel,accl,profile,justRotate);
				
//				 Broadcast the linearpath
				pose_path_broadcaster(&posVec);
				
				actualTcpPose=end;
				PoseToRPY(end,roll,pitch,yaw);
//				ROS_INFO("Endpose roll: %f pitch: %f yaw: %f",roll,pitch,yaw);
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
                TIP.circular_interpolation(posVec,x_center,y_center,z_center,roll,pitch,yaw,startAngle,endAngle,r,vel,accl,profile);
				
				move_ptp(posVec.at(0),0.03);
				pose_path_broadcaster(&posVec);	
				actualTcpPose=posVec.at(posVec.size()-1);
			}
			
			if("hold" == movement){
				actualTcpPose = getEndeffectorPose();
				ROS_INFO("Hold position");
				holdTime = atof(child->Attribute( "time"));
				ros::Timer timer = nh_.createTimer(ros::Duration(holdTime), &CartesianController::timerCallback, this);
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

void CartesianController::timerCallback(const ros::TimerEvent& event)
{
	hold_=false;
}

// Pseudo PTP
void CartesianController::move_ptp(geometry_msgs::Pose targetPose, double epsilon)
{
	reached_pos_=false;
	int reached_pos_counter=0;
	double ro,pi,ya;
	ros::Rate rate(update_rate_);
	tf::StampedTransform stampedTransform;
	tf::Quaternion q;
	bool transformed=false;
	
	while(ros::ok())
	{
		// Linearkoordinaten
		transform_.setOrigin( tf::Vector3(targetPose.position.x,targetPose.position.y,targetPose.position.z) );
	
		q = tf::Quaternion(targetPose.orientation.x,targetPose.orientation.y,targetPose.orientation.z,targetPose.orientation.w);
		transform_.setRotation(q);

		// Send br Frame
		br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, targetFrame_));
		
		// Get transformation
		try
		{
			listener_.lookupTransform(targetFrame_,chain_tip_link_, ros::Time(0), stampedTransform);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
		}
		
		// Get current RPY out of quaternion
		tf::Quaternion quatern = stampedTransform.getRotation();
		tf::Matrix3x3(quatern).getRPY(ro,pi,ya);
				
				
		// Wait for arm_7_link to be in position
		if(epsilon_area(stampedTransform.getOrigin().x(), stampedTransform.getOrigin().y(), stampedTransform.getOrigin().z(),ro,pi,ya,epsilon))
		{
			reached_pos_counter++;	// Count up if end effector position is in the epsilon area to avoid wrong values
		}
		
		if(reached_pos_counter>=50)
		{
			reached_pos_=true;
		}
		
		if(reached_pos_==true)	// Cancel while loop
		{
			break;
		}
		rate.sleep();
		ros::spinOnce();
	}
}

void CartesianController::pose_path_broadcaster(std::vector <geometry_msgs::Pose> *poseVector)
{
	ros::Rate rate(update_rate_);
	tf::Quaternion q;
	double T_IPO=pow(update_rate_,-1);
	
	for(int i=0; i<poseVector->size()-1; i++)
	{
		// Linearkoordinaten
		transform_.setOrigin(tf::Vector3(poseVector->at(i).position.x, poseVector->at(i).position.y, poseVector->at(i).position.z));
		
		q = tf::Quaternion(poseVector->at(i).orientation.x,poseVector->at(i).orientation.y,poseVector->at(i).orientation.z,poseVector->at(i).orientation.w);
		
		transform_.setRotation(q);   
		
		showMarker(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, targetFrame_),marker1_, 0 , 1.0 , 0 ,"goalFrame");
		
		br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), referenceFrame_, targetFrame_));
		
		marker1_++;
		
		ros::spinOnce();
		rate.sleep();
	}
}

void CartesianController::hold_position(geometry_msgs::Pose holdPose)
{
	ros::Rate rate(update_rate_);
	tf::Quaternion q;
	
	while(hold_)
	{
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






geometry_msgs::Pose CartesianController::getEndeffectorPose()
{
	geometry_msgs::Pose pos;
	tf::StampedTransform stampedTransform;
	bool transformed=false;
	
	do{
		// Get transformation
		try{
			listener_.lookupTransform(referenceFrame_,chain_tip_link_, ros::Time(0), stampedTransform);
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
bool CartesianController::epsilon_area(double x,double y, double z, double roll, double pitch, double yaw,double epsilon)
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
	
	if(x_okay && y_okay && z_okay && roll_okay && pitch_okay && yaw_okay)
	{
		return true;
	}else
	{
		return false;
	}
}

void CartesianController::showMarker(tf::StampedTransform tf,int marker_id,double red, double green, double blue,std::string ns)
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

void CartesianController::showDot(double x,double y,double z,int marker_id,double red, double green, double blue,std::string ns)
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

void CartesianController::showLevel(tf::Transform pos,int marker_id,double red, double green, double blue,std::string ns)
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

void CartesianController::start_tracking()
{
	cob_srvs::SetString start;
	start.request.data = targetFrame_;
	startTracking_.call(start);
	
	if(start.response.success==true)
	{
		ROS_INFO("...service called!");
	}
	else
	{
		ROS_INFO("...service failed");
	}
}

void CartesianController::stop_tracking()
{
	std_srvs::Empty srv_save_stop;
	srv_save_stop.request;
	if(stopTracking_.call(srv_save_stop))
	{
		ROS_INFO("... service stopped!");
	}
	else
	{
		ROS_ERROR("... service stop failed! FATAL!");
	}
}



void CartesianController::PoseToRPY(geometry_msgs::Pose pose,double &roll, double &pitch, double &yaw)
{
	tf::Quaternion q = tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
	tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
}
