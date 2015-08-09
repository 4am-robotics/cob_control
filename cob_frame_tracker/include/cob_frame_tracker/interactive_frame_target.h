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
#ifndef INTERACTIVE_FRAME_TARGET_H
#define INTERACTIVE_FRAME_TARGET_H

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <cob_srvs/SetString.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>



class InteractiveFrameTarget
{
public:
    InteractiveFrameTarget() {;}
    ~InteractiveFrameTarget() {;}

    bool initialize();

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

private:
    ros::Timer timer_;
    double update_rate_;

    std::string chain_tip_link_;    //twists with respect to this frame
    std::string tracking_frame_;    //goal frame
    std::string root_frame_;

    bool movable_trans_;
    bool movable_rot_;

    interactive_markers::InteractiveMarkerServer* ia_server_;
    visualization_msgs::InteractiveMarker int_marker_;
    visualization_msgs::InteractiveMarker int_marker_menu_;
    interactive_markers::MenuHandler menu_handler_;

    bool tracking_;
    tf::StampedTransform target_pose_;
    boost::mutex mutex_;

    ros::ServiceClient start_tracking_client_;
    ros::ServiceClient stop_tracking_client_;

    void updateMarker();
    void sendTransform(const ros::TimerEvent& event);
    void startTracking( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
    void stopTracking( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
    void resetTracking( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
    void menuFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
    void markerFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback );
};

#endif
