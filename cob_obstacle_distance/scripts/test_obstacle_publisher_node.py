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

import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh


if __name__ == "__main__":
    rospy.init_node("simple_obstacle_pub")
    root_frame = "/odom_combined"

    pub = rospy.Publisher("obstacle_distance/registerObstacle", CollisionObject, queue_size = 1)

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
    #x.operation = CollisionObject.REMOVE
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions.append(0.1)  # radius
    x.primitives.append(sphere)

    pose = Pose()
    pose.position.x = 0.35
    pose.position.y = -0.45
    pose.position.z = 0.8
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    x.primitive_poses.append(pose)
    pub.publish(x)
    time.sleep(1.0)

    # Now publish a mesh and use the db field for the stl-resource name
    y = CollisionObject()
    y.id = "Funny Mesh"
    y.header.frame_id = root_frame
    #y.type.db = "package://cob_gazebo_objects/Media/models/milk.dae"
    y.type.db = "package://cob_twist_controller/files/torus_0_25_inner_rad.stl"
    y.operation = CollisionObject.ADD
    #y.operation = CollisionObject.REMOVE

    pose = Pose()
    pose.position.x = 0.25
    pose.position.y = -0.60
    pose.position.z = 0.95
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    y.mesh_poses.append(pose)
    pub.publish(y)
    time.sleep(1.0)

    rospy.spin()
