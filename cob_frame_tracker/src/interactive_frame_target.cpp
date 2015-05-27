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
 *   ROS package name: cob_frame_tracker
 *
 * \author
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This class provides an interactive_marker server for specifying a tf-frame to be tracked
 *
 ****************************************************************/
#include <ros/ros.h>

#include <cob_frame_tracker/interactive_frame_target.h>


bool InteractiveFrameTarget::initialize()
{
	ros::NodeHandle nh_tracker("frame_tracker");
	
	///get params
	if (nh_tracker.hasParam("update_rate"))
	{	nh_tracker.getParam("update_rate", update_rate_);	}
	else
	{	update_rate_ = 68.0;	}	//hz
	
	if (nh_tracker.hasParam("chain_tip_link"))
	{
		nh_tracker.getParam("chain_tip_link", chain_tip_link_);
	}
	else
	{
		ROS_ERROR("No chain_tip_link specified. Aborting!");
		return false;
	}
	if (nh_tracker.hasParam("tracking_frame"))
	{
		nh_tracker.getParam("tracking_frame", tracking_frame_);
	}
	else
	{
		ROS_ERROR("No tracking_frame specified. Aborting!");
		return false;
	}
	if (nh_tracker.hasParam("root_frame"))
	{
		nh_tracker.getParam("root_frame", root_frame_);
	}
	else
	{
		ROS_ERROR("No root_frame specified. Setting to 'base_link'!");
		root_frame_ = "base_link";
	}
	
	
	if (nh_tracker.hasParam("movable_trans"))
	{	nh_tracker.getParam("movable_trans", movable_trans_);	}
	else
	{	movable_trans_ = true;	}
	if (nh_tracker.hasParam("movable_rot"))
	{	nh_tracker.getParam("movable_rot", movable_rot_);	}
	else
	{	movable_rot_ = true;	}
	
	tracking_ = false;
	
	start_tracking_client_ = nh_tracker.serviceClient<cob_srvs::SetString>("start_tracking");
	ROS_INFO("Waiting for StartTrackingServer...");
	start_tracking_client_.waitForExistence();
	ROS_INFO("...done");
	
	stop_tracking_client_ = nh_tracker.serviceClient<std_srvs::Empty>("stop_tracking");
	ROS_INFO("Waiting for StopTrackingServer...");
	stop_tracking_client_.waitForExistence();
	ROS_INFO("...done");
	
	bool transform_available = false;
	while(!transform_available)
	{
		try{
			tf_listener_.lookupTransform(root_frame_, chain_tip_link_, ros::Time(), target_pose_);
			transform_available = true;
		}
		catch (tf::TransformException ex){
			//ROS_WARN("Waiting for transform...(%s)",ex.what());
			ros::Duration(0.1).sleep();
		}
	}
	
	ia_server_ = new interactive_markers::InteractiveMarkerServer("marker_server", "", false);
	
	int_marker_.header.frame_id = root_frame_;
	int_marker_.pose.position.x = target_pose_.getOrigin().x();
	int_marker_.pose.position.y = target_pose_.getOrigin().y();
	int_marker_.pose.position.z = target_pose_.getOrigin().z();
	int_marker_.pose.orientation.x = target_pose_.getRotation().getX();
	int_marker_.pose.orientation.y = target_pose_.getRotation().getY();
	int_marker_.pose.orientation.z = target_pose_.getRotation().getZ();
	int_marker_.pose.orientation.w = target_pose_.getRotation().getW();
	int_marker_.name = "interactive_target";
	//int_marker_.description = tracking_frame_;
	int_marker_.scale = 0.5;
	
	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible = true;
	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	visualization_msgs::Marker box_marker;
	box_marker.type = visualization_msgs::Marker::CUBE;
	box_marker.scale.x = 0.1;
	box_marker.scale.y = 0.1;
	box_marker.scale.z = 0.1;
	box_marker.color.r = 0.0;
	box_marker.color.g = 0.5;
	box_marker.color.b = 0.5;
	box_marker.color.a = 1.0;
	visualization_msgs::InteractiveMarkerControl control_3d;
	control_3d.always_visible = true;
	if(movable_trans_)
	{
		control.name = "move_x";
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		int_marker_.controls.push_back(control);
		control.name = "move_y";
		control.orientation.x = 0;
		control.orientation.y = 1;
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		int_marker_.controls.push_back(control);
		control.name = "move_z";
		control.orientation.y = 0;
		control.orientation.z = 1;
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
		int_marker_.controls.push_back(control);
		control_3d.name = "move_3D";
		control_3d.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
	}
	if(movable_rot_)
	{
		control.orientation.w = 1;
		control.orientation.x = 1;
		control.orientation.y = 0;
		control.orientation.z = 0;
		control.name = "rotate_x";
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		int_marker_.controls.push_back(control);
		control.name = "rotate_y";
		control.orientation.x = 0;
		control.orientation.y = 1;
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		int_marker_.controls.push_back(control);
		control.name = "rotate_z";
		control.orientation.y = 0;
		control.orientation.z = 1;
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
		int_marker_.controls.push_back(control);
		control_3d.name = "rotate_3D";
		control_3d.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_3D;
	}
	if(movable_trans_ && movable_rot_)
	{
		control_3d.name = "move_rotate_3D";
		control_3d.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
	}
	control_3d.markers.push_back( box_marker );
	int_marker_.controls.push_back(control_3d);
	ia_server_->insert(int_marker_, boost::bind(&InteractiveFrameTarget::marker_fb,   this, _1));
	
	// create menu
	menu_handler_.insert( "StartTracking", boost::bind(&InteractiveFrameTarget::start_tracking,   this, _1) );
	menu_handler_.insert( "StopTracking", boost::bind(&InteractiveFrameTarget::stop_tracking,   this, _1) );
	menu_handler_.insert( "ResetTracking", boost::bind(&InteractiveFrameTarget::reset_tracking,   this, _1) );
	
	int_marker_menu_.header.frame_id = tracking_frame_;
	int_marker_menu_.name = "marker_menu";
	int_marker_menu_.pose.position.y = 0.5;
	visualization_msgs::Marker menu_marker;
	menu_marker.scale.x = 0.15;
	menu_marker.scale.y = 0.15;
	menu_marker.scale.z = 0.15;
	//menu_marker.type = visualization_msgs::Marker::SPHERE;
	menu_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	menu_marker.text = nh_.getNamespace() + "_menu";
	menu_marker.color.r = 1.0;
	menu_marker.color.g = 1.0;
	menu_marker.color.b = 0.0;
	menu_marker.color.a = 1.0;
	
	visualization_msgs::InteractiveMarkerControl control_menu;
	control_menu.always_visible = true;
	control_menu.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
	control_menu.name = "menu_control";
	//control_menu.description = "InteractiveMenu";
	control_menu.markers.push_back(menu_marker);
	int_marker_menu_.controls.push_back(control_menu);
	ia_server_->insert(int_marker_menu_, boost::bind(&InteractiveFrameTarget::menu_fb,   this, _1) );
	menu_handler_.apply( *ia_server_, int_marker_menu_.name );
	ia_server_->applyChanges();
	
	timer_ = nh_.createTimer(ros::Duration(1/update_rate_), &InteractiveFrameTarget::send_transform,   this );
	timer_.start();
	
	ROS_INFO("INTERACTIVE_MARKER...initialized!");
	return true;
}

void InteractiveFrameTarget::send_transform(const ros::TimerEvent& event)
{
	if(!tracking_)
	{	update_marker();	}
	
	target_pose_.stamp_ = ros::Time::now();
	target_pose_.child_frame_id_ = tracking_frame_;
	tf_broadcaster_.sendTransform(target_pose_);
}

void InteractiveFrameTarget::update_marker()
{
	bool transform_available = false;
	while(!transform_available)
	{
		try{
			tf_listener_.lookupTransform(root_frame_, chain_tip_link_, ros::Time(), target_pose_);
			transform_available = true;
		}
		catch (tf::TransformException ex){
			//ROS_WARN("Waiting for transform...(%s)",ex.what());
			ros::Duration(0.1).sleep();
		}
	}
	
	geometry_msgs::Pose new_pose;
	new_pose.position.x = target_pose_.getOrigin().x();
	new_pose.position.y = target_pose_.getOrigin().y();
	new_pose.position.z = target_pose_.getOrigin().z();
	new_pose.orientation.x = target_pose_.getRotation().getX();
	new_pose.orientation.y = target_pose_.getRotation().getY();
	new_pose.orientation.z = target_pose_.getRotation().getZ();
	new_pose.orientation.w = target_pose_.getRotation().getW();
	ia_server_->setPose(int_marker_.name, new_pose);
	ia_server_->applyChanges();
}

void InteractiveFrameTarget::start_tracking( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	ROS_INFO("StartTracking");
	
	cob_srvs::SetString srv;
	srv.request.data = tracking_frame_;
	
	if(start_tracking_client_.call(srv))
	{
		tracking_ = true;
	}
	else
	{
		ROS_ERROR("Service call failed");
		tracking_ = false;
	}
}

void InteractiveFrameTarget::stop_tracking( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	std_srvs::Empty srv;
	
	if(stop_tracking_client_.call(srv))
	{
		tracking_ = false;
	}
	else
	{
		ROS_ERROR("Service call failed");
		tracking_ = false;
	}
}

void InteractiveFrameTarget::reset_tracking( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	stop_tracking(feedback);
	update_marker();
}

void InteractiveFrameTarget::menu_fb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	return;
}

void InteractiveFrameTarget::marker_fb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	target_pose_.stamp_ = feedback->header.stamp;
	target_pose_.frame_id_ = feedback->header.frame_id;
	target_pose_.child_frame_id_ = tracking_frame_;
	target_pose_.setOrigin(tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
	target_pose_.setRotation(tf::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w));
	
	ia_server_->applyChanges();
}
