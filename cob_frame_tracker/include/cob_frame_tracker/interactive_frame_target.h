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


#ifndef INTERACTIVE_FRAME_TARGET_H
#define INTERACTIVE_FRAME_TARGET_H

#include <string>
#include <ros/ros.h>

#include <std_srvs/Trigger.h>
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

    std::string root_frame_;
    std::string chain_tip_link_;
    std::string tracking_frame_;    // the frame tracking the target
    std::string target_frame_;      // the frame to be tracked

    bool movable_trans_;
    bool movable_rot_;

    interactive_markers::InteractiveMarkerServer* ia_server_;
    visualization_msgs::InteractiveMarker int_marker_;
    visualization_msgs::InteractiveMarker int_marker_menu_;
    interactive_markers::MenuHandler menu_handler_;

    bool tracking_;
    bool lookat_;
    tf::StampedTransform target_pose_;
    boost::mutex mutex_;

    ros::ServiceClient start_tracking_client_;
    ros::ServiceClient start_lookat_client_;
    ros::ServiceClient stop_client_;

    void updateMarker(const std::string& frame);
    void sendTransform(const ros::TimerEvent& event);
    void startTracking(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void startLookat(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void stop(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void menuFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void markerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};

#endif
