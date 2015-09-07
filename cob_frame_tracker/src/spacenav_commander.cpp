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

#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>


class SpacenavCommander
{
public:
    SpacenavCommander()
    {
        twist_spacenav_sub_ = nh_.subscribe("/spacenav/twist", 1, &SpacenavCommander::twistSpacenavCallback, this);
        twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped> ("twist_controller/command_twist_stamped", 1);
        
        if (nh_.hasParam("root_frame"))
        {
            nh_.getParam("root_frame", root_frame_);
        }
        else
        {
            ROS_ERROR("No root_frame specified. Setting to 'base_link'!");
            root_frame_ = "base_link";
        }
    }

    ~SpacenavCommander()
    {
    }

    /// Receive Twist from a 6d input device (e.g. 3DConnexion SpaceNavigator)
    void twistSpacenavCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        KDL::Twist twist;
        tf::twistMsgToKDL(*msg, twist);
        geometry_msgs::TwistStamped ts;
        
        if(twist != KDL::Twist::Zero())
        {
            ts.header.frame_id = root_frame_;
            ts.header.stamp = ros::Time::now();
            ts.twist = *msg,
            twist_pub_.publish(ts);
        }
        else
        {
            ROS_DEBUG("ZeroTwist received");
        }
    }
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber twist_spacenav_sub_;
    ros::Publisher twist_pub_;
    std::string root_frame_;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "spacenav_commander");
    SpacenavCommander sc;
    ros::spin();
}
