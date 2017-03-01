#!/usr/bin/env python
"""
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
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: July, 2015
 *
 * \brief
 *   Simple Python node to collect data from several topics.
 *   Subscription and how data is written to a file is done in data_collection.py module.
 *
"""
import time
import rospy
import subprocess

from simple_script_server.simple_script_server import simple_script_server
import twist_controller_config as tcc
from dynamic_reconfigure.client import Client
from visualization_msgs.msg import *
from sensor_msgs.msg import JointState

from data_collection import JointStateDataKraken
from data_collection import TwistDataKraken
from data_collection import JointVelocityDataKraken
from data_collection import FrameTrackingDataKraken

# has to be startet with ns param: rosrun cob_twist_controller collect_twist_control_eval_data.py __ns:=arm_right
###########################################################################
################ Auxilliary functions #####################################
# creates a geometry_msgs::PoseStamped from x,y,z and yaw, pitch, roll
def createPose(x, y, z, yaw, pitch, roll, frame_id):
    pose = geometry_msgs.msg.PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time(0)
    return pose

def talker():
    rospy.init_node('talker', anonymous=True)
    publisher_marker_object_name = rospy.Publisher("/arm_left/marker_server/feedback", InteractiveMarkerFeedback)

    marker_object_name = InteractiveMarkerFeedback()
    marker_object_name.header.frame_id = "odom_combined"
    marker_object_name.header.stamp = rospy.Time.now()
    marker_object_name.marker_name = "interactive_target"
    marker_object_name.control_name = "move_rotate_3D"
    marker_object_name.event_type = 1
    marker_object_name.pose.position.x = 0.0
    marker_object_name.pose.position.y = 0.996738
    marker_object_name.pose.position.z = 1.0064
    marker_object_name.pose.orientation.x = 0.0
    marker_object_name.pose.orientation.y = 0.70717
    marker_object_name.pose.orientation.z = 0.70705
    marker_object_name.pose.orientation.w = 0.0
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(2)
    t0 = rospy.get_time()
    t1 = rospy.get_time()
    while t1-t0<5.0:
        t1 = rospy.get_time()
        publisher_marker_object_name.publish(marker_object_name)

    t0 = rospy.get_time()
    while t1-t0<5.0:
        t1 = rospy.get_time()
        marker_object_name.pose.position.x = 0.7
        marker_object_name.pose.position.y = -0.35
        publisher_marker_object_name.publish(marker_object_name)

    t0 = rospy.get_time()
    while t1-t0<5.0:
        t1 = rospy.get_time()
        marker_object_name.pose.position.x = 0.0
        marker_object_name.pose.position.y = 0.996738
        publisher_marker_object_name.publish(marker_object_name)

    t0 = rospy.get_time()
    while t1-t0<5.0:
        t1 = rospy.get_time()
        marker_object_name.pose.position.x = -0.7
        marker_object_name.pose.position.y = -0.35
        publisher_marker_object_name.publish(marker_object_name)

    t0 = rospy.get_time()
    while t1-t0< 5.0:
        t1 = rospy.get_time()
        marker_object_name.pose.position.x = 0.0
        marker_object_name.pose.position.y = 0.996738
        publisher_marker_object_name.publish(marker_object_name)
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass