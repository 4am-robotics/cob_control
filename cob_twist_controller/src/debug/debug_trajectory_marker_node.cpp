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
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_twist_controller/TwistControllerConfig.h>

class DebugTrajectoryMarker
{
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    tf::TransformListener tf_listener_;

    ros::Timer marker_timer_;

    std::string root_frame_;
    std::string base_link_;
    std::string chain_tip_link_;
    std::string target_frame_;

    visualization_msgs::Marker tip_marker_;
    visualization_msgs::Marker target_marker_;
    visualization_msgs::Marker base_marker_;

public:
    int init()
    {
        if (!nh_.getParam("root_frame", this->root_frame_))
        {
            ROS_ERROR("Failed to get parameter \"root_frame\".");
            return -1;
        }

        this->base_link_ = "base_link";

        if (!nh_.getParam("chain_tip_link", this->chain_tip_link_))
        {
            ROS_ERROR("Failed to get parameter \"chain_tip_link\".");
            return -2;
        }

        if (!nh_.getParam("frame_tracker/target_frame", this->target_frame_))
        {
            ROS_ERROR_STREAM("Please provide a 'frame_tracker/target_frame' parameter in this node's namespace.");
            return -3;
        }

        marker_pub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("trajectory_marker", 1, true);
        marker_timer_ = this->nh_.createTimer(ros::Duration(0.1), &DebugTrajectoryMarker::publishMarker, this);

        while (marker_pub_.getNumSubscribers() < 1)
        {
            if (ros::isShuttingDown())
            {
                return -4;
            }
            ROS_WARN_STREAM_ONCE("Please create a subscriber to '" + this->nh_.getNamespace() + "/trajectory_marker' topic (Type: visualization_msgs/MarkerArray)");
            ros::Duration(1.0).sleep();
        }

        std_msgs::ColorRGBA green;
        green.g = green.a = 1.0;
        tip_marker_ = getMarker(green);
        tip_marker_.ns = "tip_marker";

        std_msgs::ColorRGBA red;
        red.r = red.a = 1.0;
        target_marker_ = getMarker(red);
        target_marker_.ns = "target_marker";

        std_msgs::ColorRGBA blue;
        blue.b = blue.a = 1.0;
        base_marker_ = getMarker(blue);
        base_marker_.ns = "base_marker";

        marker_timer_.start();

        return 0;
    }

    void publishMarker(const ros::TimerEvent& event)
    {
        visualization_msgs::MarkerArray marker_array;
        geometry_msgs::Point point;

        if (tf_listener_.frameExists(this->chain_tip_link_))
        {
            if (tip_marker_.points.size() > 10000)
            {
                tip_marker_.points.clear();
            }
            point = getPoint(this->root_frame_, this->chain_tip_link_);
            tip_marker_.points.push_back(point);
            marker_array.markers.push_back(tip_marker_);
        }

        if (tf_listener_.frameExists(this->target_frame_))
        {
            if (target_marker_.points.size() > 10000)
            {
                target_marker_.points.clear();
            }
            point = getPoint(this->root_frame_, this->target_frame_);
            target_marker_.points.push_back(point);
            marker_array.markers.push_back(target_marker_);
        }

        if (tf_listener_.frameExists(this->base_link_))
        {
            if (nh_.param("twist_controller/kinematic_extension", 0) == cob_twist_controller::TwistController_BASE_ACTIVE)
            {
                if (base_marker_.points.size() > 10000)
                {
                    base_marker_.points.clear();
                }
                point = getPoint(this->root_frame_, this->base_link_);
                base_marker_.points.push_back(point);
                marker_array.markers.push_back(base_marker_);
            }
        }

        if (!marker_array.markers.empty())
        {
            this->marker_pub_.publish(marker_array);
        }
    }

    visualization_msgs::Marker getMarker(std_msgs::ColorRGBA color)
    {
        visualization_msgs::Marker box_marker;
        box_marker.type = visualization_msgs::Marker::LINE_STRIP;
        box_marker.lifetime = ros::Duration();
        box_marker.action = visualization_msgs::Marker::ADD;
        box_marker.id = 42;
        box_marker.header.stamp = ros::Time::now();
        box_marker.header.frame_id = this->root_frame_;

        box_marker.scale.x = 0.01;
        box_marker.color = color;
        box_marker.pose.orientation.w = 1.0;

        return box_marker;
    }

    geometry_msgs::Point getPoint(std::string from, std::string to)
    {
        geometry_msgs::Point point;
        tf::StampedTransform transform;
        try
        {
            ros::Time now = ros::Time::now();
            tf_listener_.waitForTransform(from, to, now, ros::Duration(0.5));
            tf_listener_.lookupTransform(from, to, now, transform);

            point.x = transform.getOrigin().x();
            point.y = transform.getOrigin().y();
            point.z = transform.getOrigin().z();
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        return point;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "debug_trajectory_marker_node");

    DebugTrajectoryMarker dtm;
    if (dtm.init() != 0)
    {
        ROS_ERROR("Failed to initialize DebugTrajectoryMarker.");
        return -1;
    }

    ros::spin();
}
