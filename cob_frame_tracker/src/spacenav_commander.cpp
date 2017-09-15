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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

#include <boost/thread/mutex.hpp>

class SpacenavCommander
{
public:
    SpacenavCommander()
    {
        twist_spacenav_sub_ = nh_.subscribe("/spacenav/twist", 1, &SpacenavCommander::twistSpacenavCallback, this);
        joy_spacenav_sub_ = nh_.subscribe("/spacenav/joy", 1, &SpacenavCommander::joySpacenavCallback, this);
        twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped> ("twist_controller/command_twist_stamped", 1);

        if (nh_.hasParam("root_frame"))
        {
            nh_.getParam("root_frame", root_frame_);
        }
        else
        {
            ROS_WARN("No 'root_frame' specified. Setting to 'base_link'!");
            root_frame_ = "base_link";
        }
        if (nh_.hasParam("chain_tip_link"))
        {
            nh_.getParam("chain_tip_link", tip_frame_);
        }
        else
        {
            ROS_ERROR("No 'chain_tip_link' specified. Using 'root_frame' instead!");
            tip_frame_ = root_frame_;
        }

        // could be made a parameter or even dynamically reconfigurable
        ros::NodeHandle nh_priv("~");
        nh_priv.param<double>("scaling_factor", scaling_factor_, 0.1);
        dead_man_enabled_ = false;
        frame_id_ = root_frame_;

        timer_ = nh_.createTimer(ros::Duration(0.01), &SpacenavCommander::timerCallback, this);  // guarantee 100Hz
        timer_.start();
    }

    ~SpacenavCommander()
    {
    }

    /// Receive Joy from a 6d input device (e.g. 3DConnexion SpaceNavigator)
    void joySpacenavCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(mutex_);

        if (msg->buttons[0])  // dead man
        {
            dead_man_enabled_ = true;
        }
        else
        {
            // publish one last zero-twist
            if (dead_man_enabled_)
            {
                geometry_msgs::TwistStamped ts;
                ts.header.frame_id = root_frame_;
                ts.header.stamp = ros::Time::now();
                twist_pub_.publish(ts);
            }
            dead_man_enabled_ = false;
        }

        if (msg->buttons[1])  // root_frame/tip_frame selection
        {
            frame_id_ = root_frame_;
        }
        else
        {
            frame_id_ = tip_frame_;
        }
    }

    /// Receive Twist from a 6d input device (e.g. 3DConnexion SpaceNavigator)
    void twistSpacenavCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(mutex_);

        ts_.twist.linear.x = scaling_factor_ * msg->linear.x;
        ts_.twist.linear.y = scaling_factor_ * msg->linear.y;
        ts_.twist.linear.z = scaling_factor_ * msg->linear.z;
        ts_.twist.angular.x = scaling_factor_ * msg->angular.x;
        ts_.twist.angular.y = scaling_factor_ * msg->angular.y;
        ts_.twist.angular.z = scaling_factor_ * msg->angular.z;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        boost::mutex::scoped_lock lock(mutex_);

        if (dead_man_enabled_)
        {
            ts_.header.frame_id = frame_id_;
            ts_.header.stamp = ros::Time::now();

            twist_pub_.publish(ts_);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber twist_spacenav_sub_;
    ros::Subscriber joy_spacenav_sub_;
    ros::Publisher twist_pub_;
    ros::Timer timer_;

    bool dead_man_enabled_;
    boost::mutex mutex_;
    double scaling_factor_;
    std::string root_frame_, tip_frame_, frame_id_;
    geometry_msgs::TwistStamped ts_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "spacenav_commander");
    SpacenavCommander sc;
    ros::spin();
}
