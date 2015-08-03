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
 *   ROS package name: cob_obstacle_distance
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: July, 2015
 *
 * \brief
 *   Simple Python node to publish obstacles to cob_obstacle_distance node.
 *
"""
import time
import roslib 
import rospy

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from shape_msgs.msg import Mesh


if __name__=="__main__":
    rospy.init_node("simple_obstacle_pub")
    root_frame = "/odom_combined"
        
    pub = rospy.Publisher("/arm_right/obstacle_distance/registerObstacle", CollisionObject, queue_size=1)
       
    while pub.get_num_connections() < 1: 
        if rospy.is_shutdown():
            exit(0)
        rospy.logwarn("Please create a subscriber to '/arm_right/obstacle_distance/registerObstacle' topic (Type: moveit_msgs/CollisionObject)")
        time.sleep(1.0)
    
    rospy.loginfo("Continue ...")
    
    # Publish a simple sphere
    x = CollisionObject()
    x.id = "Funny Sphere"
    x.header.frame_id = root_frame
    x.operation = CollisionObject.ADD
    
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions.append(0.1) # radius
    x.primitives.append(sphere)
    
    pose = Pose()
    pose.position.x = 0.35
    pose.position.y = -0.35
    pose.position.z = 0.8
    pose.orientation.x = 0.0; 
    pose.orientation.y = 0.0; 
    pose.orientation.z = 0.0; 
    pose.orientation.w = 1.0; 
    x.primitive_poses.append(pose)
    
    pub.publish(x)
    
    # Now publish a mesh and use the db field for the stl-resource name
    # This is an example how to use STL-files
    # Other possibility: Full usage would be to parse the STL file here, build the triangles and publish it. 
    # But then rviz won't show a mesh marker!
    time.sleep(1.5)
    y = CollisionObject()
    y.id = "Funny Mesh"
    y.header.frame_id = root_frame
    y.type.db = "package://cob_gazebo_objects/Media/models/milk.dae"
    y.operation = CollisionObject.ADD
    
    pose = Pose()
    pose.position.x = -0.35
    pose.position.y = -0.35
    pose.position.z = 0.8
    pose.orientation.x = 0.0; 
    pose.orientation.y = 0.0; 
    pose.orientation.z = 0.0; 
    pose.orientation.w = 1.0; 
    y.mesh_poses.append(pose)

    pub.publish(y)
    
    rospy.spin()
