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
    box_marker.scale.x = 0.01 # only this used for line_strip
    #box_marker.scale.y = 0.2
    #box_marker.scale.z = 0.1
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
    rospy.init_node("trajectory_marker_publisher")
    
    listener = tf.TransformListener()
    
    root_frame = rospy.get_param('root_frame')
    base_link = "base_link"
    chain_tip_link = rospy.get_param('chain_tip_link')
    tracking_frame = rospy.get_param('frame_tracker/tracking_frame')
    
    pub = rospy.Publisher('trajectory_marker', Marker, queue_size=1)
    
    colorx = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    tip_marker = getMarker(root_frame, 101, colorx)
    
    colorx = ColorRGBA(0.0, 1.0, 0.0, 1.0)
    target_marker = getMarker(root_frame, 102, colorx)
    
    colorx = ColorRGBA(0.0, 0.0, 1.0, 1.0)
    base_marker = getMarker(root_frame, 103, colorx)
    
    
    while pub.get_num_connections() < 1: 
        if rospy.is_shutdown():
            exit(0)
        rospy.logwarn("Please create a subscriber to '" + rospy.get_namespace() + "/trajectory_marker' topic (Type: visualization_msgs/Marker)")
        time.sleep(1.0)
    
    rate = rospy.Rate(20) # 20hz
    
    counter = 0
    while not rospy.is_shutdown():
        try:
            (trans_tip, rot_tip) = listener.lookupTransform(root_frame, chain_tip_link, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            continue

        try:
            (trans_target, rot_target) = listener.lookupTransform(root_frame, tracking_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            continue
        
        base_active = rospy.get_param('twist_controller/kinematic_extension')
        if(base_active == 1):
            try:
                (trans_base, rot_base) = listener.lookupTransform(root_frame, base_link, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(str(e))
                continue
        
        # ######################## TipMarker ###############################
        p = Point()
        p.x = trans_tip[0]
        p.y = trans_tip[1]
        p.z = trans_tip[2]
        
        if counter > 100000:
            counter = 0
            last_point = tip_marker.points.pop()
            tip_marker.points = []
            tip_marker.id = tip_marker.id + 1
            tip_marker.points.append(last_point)
        
        tip_marker.points.append(p)
        pub.publish(tip_marker)
        time.sleep(0.001)
        
        # ######################## TargetMarker ###############################
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
        pub.publish(target_marker)
        time.sleep(0.001)
        
        # ######################## BaseMarker ###############################
        if(base_active == 1):
            p = Point()
            p.x = trans_base[0]
            p.y = trans_base[1]
            p.z = trans_base[2]
            
            if counter > 100000:
                counter = 0
                last_point = base_marker.points.pop()
                base_marker.points = []
                base_marker.id = base_marker.id + 1
                base_marker.points.append(last_point)
            
            base_marker.points.append(p)
            pub.publish(base_marker)
            time.sleep(0.001)
        
        # ######################################################################
        counter += 1
        try:
            rate.sleep()
        except rospy.ROSInterruptException as e:
            pass
