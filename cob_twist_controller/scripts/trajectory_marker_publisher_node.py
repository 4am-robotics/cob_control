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
 *   Simple Python node to publish a line strip to see the real trajectory and the desired one. 
 *
"""

import time
import roslib; roslib.load_manifest("interactive_markers")
import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import *
import tf


def getMarker(root_frame, start_id, color):
    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.LINE_STRIP
    box_marker.scale.x = 0.03 # only this used for line_strip
#    box_marker.scale.y = 0.2
#    box_marker.scale.z = 0.1
    box_marker.color.r = color.r
    box_marker.color.g = color.g
    box_marker.color.b = color.b
    box_marker.color.a = color.a
    
    box_marker.pose.position.x = 0.0
    box_marker.pose.position.y = 0.0
    box_marker.pose.position.z = 0.0
    box_marker.pose.orientation.x = 0.0
    box_marker.pose.orientation.y = 0.0
    box_marker.pose.orientation.z = 0.0
    box_marker.pose.orientation.w = 1.0
    
    box_marker.ns = "basic_shapes"
    box_marker.id = start_id
    box_marker.action = Marker.ADD
    box_marker.header.frame_id = root_frame
    box_marker.header.stamp = rospy.Time.now()
    
    box_marker.lifetime = rospy.Duration()
    return box_marker


if __name__=="__main__":
    rospy.init_node("simple_marker")
    
    listener = tf.TransformListener()
    
    root_frame = "/odom_combined"
    arm_def = "/arm_right"
    
    
    
    
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
       
    colorx = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    box_marker = getMarker(root_frame, 99, colorx)
    
    colorx = ColorRGBA(0.0, 1.0, 0.0, 1.0)
    target_marker = getMarker(root_frame, 999, colorx)
    

    while pub.get_num_connections() < 1: 
        if rospy.is_shutdown():
            exit(0)
        rospy.logwarn("Please create a subscriber to the marker")
        time.sleep(1.0)
    
    
    
    rate = rospy.Rate(20) # 10hz
    
    counter = 0
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(root_frame, arm_def + '_7_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            continue

        try:
            (trans_target, rot_target) = listener.lookupTransform(root_frame, arm_def + '_7_target', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            continue
# ######################################################################
        p = Point()
        p.x = trans[0]
        p.y = trans[1]
        p.z = trans[2]
        
        if counter > 100000:
            counter = 0
            last_point = box_marker.points.pop()
            box_marker.points = []
            box_marker.id = box_marker.id + 1
            box_marker.points.append(last_point)
        
        box_marker.points.append(p)
# ######################################################################
        p = Point()
        p.x = trans_target[0]
        p.y = trans_target[1]
        p.z = trans_target[2]
        
        if counter > 100000:
            counter = 0
            last_point = target_marker.points.pop()
            target_marker.points = []
            target_marker.id = target_marker.id + 1
            target_marker.points.append(last_point)
        
        target_marker.points.append(p)
# ######################################################################
#         box_marker.pose.position.x = trans[0]
#         box_marker.pose.position.y = trans[1]
#         box_marker.pose.position.z = trans[2]
#         box_marker.pose.orientation.x = 0.0
#         box_marker.pose.orientation.y = 0.0
#         box_marker.pose.orientation.z = 0.0
#         box_marker.pose.orientation.w = 1.0
#        box_marker.id = box_marker.id + 1
        
        rospy.loginfo("Publishing")
        pub.publish(box_marker)
        time.sleep(0.001)
        pub.publish(target_marker)
        
        counter += 1
        rate.sleep()