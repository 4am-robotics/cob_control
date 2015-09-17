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
 *   ROS package name: cob_frame_tracker
 *
 * \author
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: September, 2015
 *
 * \brief
 *   This class sends twists from a 6d input device
 *
 ****************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>
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
            ROS_WARN("No root_frame specified. Setting to 'base_link'!");
            root_frame_ = "base_link";
        }
        
        //could be made a parameter or even dynamically reconfigurable
        scaling_factor_ = 10.0;
        dead_man_enabled_ = false;
    }

    ~SpacenavCommander()
    {
    }

    /// Receive Joy from a 6d input device (e.g. 3DConnexion SpaceNavigator)
    void joySpacenavCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(mutex_);
        if(msg->buttons[0] && msg->buttons[1])
        {
            dead_man_enabled_ = true;
        }
        else
        {
            dead_man_enabled_ = false;
        }
    }

    /// Receive Twist from a 6d input device (e.g. 3DConnexion SpaceNavigator)
    void twistSpacenavCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        KDL::Twist twist;
        tf::twistMsgToKDL(*msg, twist);
        geometry_msgs::TwistStamped ts;
        
        boost::mutex::scoped_lock lock(mutex_);
        if(dead_man_enabled_)
        {
            if(twist != KDL::Twist::Zero())
            {
                ts.header.frame_id = root_frame_;
                ts.header.stamp = ros::Time::now();
                ts.twist.linear.x = msg->linear.x / scaling_factor_;
                ts.twist.linear.y = msg->linear.y / scaling_factor_;
                ts.twist.linear.z = msg->linear.z / scaling_factor_;
                ts.twist.angular.x = msg->angular.x / scaling_factor_;
                ts.twist.angular.y = msg->angular.y / scaling_factor_;
                ts.twist.angular.z = msg->angular.z / scaling_factor_;
                
                twist_pub_.publish(ts);
            }
            else
            {
                ROS_DEBUG("ZeroTwist received");
            }
        }
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber twist_spacenav_sub_;
    ros::Subscriber joy_spacenav_sub_;
    ros::Publisher twist_pub_;
    
    bool dead_man_enabled_;
    boost::mutex mutex_;
    double scaling_factor_;
    std::string root_frame_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "spacenav_commander");
    SpacenavCommander sc;
    ros::spin();
}
