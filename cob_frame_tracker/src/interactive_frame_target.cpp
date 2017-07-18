/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <string>
#include <ros/ros.h>

#include <cob_frame_tracker/interactive_frame_target.h>


bool InteractiveFrameTarget::initialize()
{
    ros::NodeHandle nh_tracker("frame_tracker");

    /// get params
    if (nh_tracker.hasParam("update_rate"))
    {    nh_tracker.getParam("update_rate", update_rate_);    }
    else
    {    update_rate_ = 50.0;    }    // hz

    if (nh_.hasParam("chain_tip_link"))
    {
        nh_.getParam("chain_tip_link", chain_tip_link_);
    }
    else
    {
        ROS_ERROR("No chain_tip_link specified. Aborting!");
        return false;
    }

    if (nh_.hasParam("root_frame"))
    {
        nh_.getParam("root_frame", root_frame_);
    }
    else
    {
        ROS_ERROR("No root_frame specified. Setting to 'base_link'!");
        root_frame_ = "base_link";
    }

    if (nh_tracker.hasParam("target_frame"))
    {
        nh_tracker.getParam("target_frame", target_frame_);
    }
    else
    {
        ROS_ERROR("No target_frame specified. Aborting!");
        return false;
    }

    if (nh_tracker.hasParam("movable_trans"))
    {    nh_tracker.getParam("movable_trans", movable_trans_);    }
    else
    {    movable_trans_ = true;    }
    if (nh_tracker.hasParam("movable_rot"))
    {    nh_tracker.getParam("movable_rot", movable_rot_);    }
    else
    {    movable_rot_ = true;    }

    tracking_ = false;
    lookat_ = false;
    tracking_frame_ = chain_tip_link_;

    start_tracking_client_ = nh_tracker.serviceClient<cob_srvs::SetString>("start_tracking");
    ROS_INFO("Waiting for StartTrackingServer...");
    start_tracking_client_.waitForExistence();

    start_lookat_client_ = nh_tracker.serviceClient<cob_srvs::SetString>("start_lookat");
    ROS_INFO("Waiting for StartLookatServer...");
    start_lookat_client_.waitForExistence();

    stop_client_ = nh_tracker.serviceClient<std_srvs::Trigger>("stop");
    ROS_INFO("Waiting for StopServer...");
    stop_client_.waitForExistence();
    ROS_INFO("InteractiveFrameTarget: All Services available!");

    bool transform_available = false;
    while (!transform_available)
    {
        try
        {
            tf_listener_.lookupTransform(root_frame_, tracking_frame_, ros::Time(), target_pose_);
            transform_available = true;
        }
        catch (tf::TransformException& ex)
        {
            // ROS_WARN("IFT::initialize: Waiting for transform...(%s)",ex.what());
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
    // int_marker_.description = target_frame_;
    int_marker_.scale = nh_tracker.param("marker_scale", 0.5);

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
    box_marker.color.a = 0.5;
    visualization_msgs::InteractiveMarkerControl control_3d;
    control_3d.always_visible = true;
    if (movable_trans_)
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
    if (movable_rot_)
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
    if (movable_trans_ && movable_rot_)
    {
        control_3d.name = "move_rotate_3D";
        control_3d.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    }
    control_3d.markers.push_back(box_marker);
    int_marker_.controls.push_back(control_3d);
    ia_server_->insert(int_marker_, boost::bind(&InteractiveFrameTarget::markerFeedback,   this, _1));

    /// create menu
    menu_handler_.insert("StartTracking", boost::bind(&InteractiveFrameTarget::startTracking,   this, _1));
    menu_handler_.insert("StartLookat", boost::bind(&InteractiveFrameTarget::startLookat,   this, _1));
    menu_handler_.insert("Stop", boost::bind(&InteractiveFrameTarget::stop,   this, _1));

    int_marker_menu_.header.frame_id = target_frame_;
    int_marker_menu_.name = "marker_menu";
    int_marker_menu_.pose.position.y = int_marker_.scale;
    visualization_msgs::Marker menu_marker;
    menu_marker.scale.x = 0.1;
    menu_marker.scale.y = 0.1;
    menu_marker.scale.z = 0.1;
    menu_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    menu_marker.text = nh_.getNamespace() + "_menu";
    menu_marker.color.r = 0.0;
    menu_marker.color.g = 0.0;
    menu_marker.color.b = 1.0;
    menu_marker.color.a = 0.85;

    visualization_msgs::InteractiveMarkerControl control_menu;
    control_menu.always_visible = true;
    control_menu.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    control_menu.name = "menu_control";
    // control_menu.description = "InteractiveMenu";
    control_menu.markers.push_back(menu_marker);
    int_marker_menu_.controls.push_back(control_menu);
    ia_server_->insert(int_marker_menu_, boost::bind(&InteractiveFrameTarget::menuFeedback,   this, _1));
    menu_handler_.apply(*ia_server_, int_marker_menu_.name);
    ia_server_->applyChanges();

    timer_ = nh_.createTimer(ros::Duration(1/update_rate_), &InteractiveFrameTarget::sendTransform,   this);
    timer_.start();

    ROS_INFO("INTERACTIVE_MARKER...initialized!");
    return true;
}

void InteractiveFrameTarget::sendTransform(const ros::TimerEvent& event)
{
    if (!tracking_ && !lookat_)
    {    updateMarker(tracking_frame_);    }

    target_pose_.stamp_ = ros::Time::now();
    target_pose_.child_frame_id_ = target_frame_;
    tf_broadcaster_.sendTransform(target_pose_);
}

void InteractiveFrameTarget::updateMarker(const std::string& frame)
{
    bool transform_available = false;
    while (!transform_available)
    {
        try
        {
            tf_listener_.lookupTransform(root_frame_, frame, ros::Time(), target_pose_);
            transform_available = true;
        }
        catch (tf::TransformException& ex)
        {
            // ROS_WARN("IFT::updateMarker: Waiting for transform...(%s)",ex.what());
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

void InteractiveFrameTarget::startTracking(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    cob_srvs::SetString srv;
    srv.request.data = target_frame_;
    bool success = start_tracking_client_.call(srv);

    if (success && srv.response.success)
    {
        ROS_INFO_STREAM("StartTracking successful: " << srv.response.message);
        tracking_ = true;
        lookat_ = false;
    }
    else
    {
        ROS_ERROR_STREAM("StartTracking failed: " << srv.response.message);
    }
}

void InteractiveFrameTarget::startLookat(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    cob_srvs::SetString srv;
    srv.request.data = target_frame_;
    bool success = start_lookat_client_.call(srv);

    if (success && srv.response.success)
    {
        ROS_INFO_STREAM("StartLookat successful: " << srv.response.message);

        updateMarker("lookat_focus_frame");

        tracking_ = false;
        lookat_ = true;
    }
    else
    {
        ROS_ERROR_STREAM("StartLookat failed: " << srv.response.message);
    }
}

void InteractiveFrameTarget::stop(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    std_srvs::Trigger srv;
    bool success = stop_client_.call(srv);

    if (success && srv.response.success)
    {
        ROS_INFO_STREAM("Stop successful: " << srv.response.message);
        tracking_ = false;
        lookat_ = false;
    }
    else
    {
        ROS_ERROR_STREAM("Stop failed: " << srv.response.message);
    }
}

void InteractiveFrameTarget::menuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    return;
}

void InteractiveFrameTarget::markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    target_pose_.stamp_ = feedback->header.stamp;
    target_pose_.frame_id_ = feedback->header.frame_id;
    target_pose_.child_frame_id_ = target_frame_;
    target_pose_.setOrigin(tf::Vector3(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z));
    target_pose_.setRotation(tf::Quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w));

    ia_server_->applyChanges();
}
