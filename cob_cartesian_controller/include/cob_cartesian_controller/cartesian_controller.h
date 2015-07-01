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

#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H

#include <vector>
#include <tinyxml.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cob_cartesian_controller/CartesianControllerAction.h>
#include <actionlib/server/simple_action_server.h>
#include <string.h>

typedef actionlib::SimpleActionServer<cob_cartesian_controller::CartesianControllerAction> tSAS_CartesianControllerAction;

struct action_type{
        std::string name;
        bool rotateOnly;
        std::string profile;
        double x, y, z, roll, pitch, yaw, vel, accl;
};

class CartesianController
{
public:
	bool initialize();
	void load();
	
	// Main functions
	void pose_path_broadcaster(std::vector <geometry_msgs::Pose> *poseVector);
	void move_ptp(geometry_msgs::Pose targetPose, double epsilon);
	void hold_position(geometry_msgs::Pose);
	
	// Helper function
	bool epsilon_area(double,double,double,double,double,double,double);
	geometry_msgs::Pose getEndeffectorPose();
	void showMarker(tf::StampedTransform,int,double,double,double,std::string);
	void showDot(double,double,double,int,double,double,double,std::string);
	void showLevel(tf::Transform,int,double,double,double,std::string);
	void timerCallback(const ros::TimerEvent&);

	void start_tracking();
	void stop_tracking();
	void PoseToRPY(geometry_msgs::Pose pose,double &roll, double &pitch, double &yaw);

    /// Action interface
    void goalCB();
    void preemptCB();

private:
	ros::NodeHandle nh_;
	
	// Publisher
	ros::Publisher vis_pub_;
	ros::Publisher path_pub_;
	ros::Publisher speed_pub_;
	ros::Publisher accl_pub_;
	ros::Publisher jerk_pub_;
	ros::ServiceClient startTracking_;
	ros::ServiceClient stopTracking_;
	
	//TF Broadcaster-Var
	tf::TransformBroadcaster br_;
	tf::Transform transform_;
	tf::Quaternion q_;
	tf::TransformListener listener_;
	tf::StampedTransform currentEndeffectorStampedTransform_;
	
	// Var for PTP Movement and hold Position
	bool reached_pos_,hold_;
	
	// yaml params
	double update_rate_;
	std::string stringPath_, fileName_;
	std::string referenceFrame_,targetFrame_;
	std::string chain_tip_link_;
	const char* charPath_;
	
	int marker1_;

    /// Action interface
    std::string action_name_;
    boost::shared_ptr<tSAS_CartesianControllerAction> as_;
    cob_cartesian_controller::CartesianControllerFeedback action_feedback_;
    cob_cartesian_controller::CartesianControllerResult action_result_;

    action_type at_;

};

#endif
