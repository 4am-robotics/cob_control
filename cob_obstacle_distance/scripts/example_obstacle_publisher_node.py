#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time

import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh


if __name__=="__main__":
    rospy.init_node("example_obstacle_publisher")

    #Specify a frame_id - transformation to root_frame of obstacle_distance node is handled in according subscriber callback
    frame_id = rospy.get_param('root_frame')

    pub = rospy.Publisher("obstacle_distance/registerObstacle", CollisionObject, queue_size=1, latch=True)

    while pub.get_num_connections() < 1:
        if rospy.is_shutdown():
            exit(0)
        rospy.logwarn("Please create a subscriber to '" + rospy.get_namespace() + "/obstacle_distance/registerObstacle' topic (Type: moveit_msgs/CollisionObject)")
        time.sleep(1.0)

    rospy.loginfo("Continue ...")

    # Publish a simple sphere
    x = CollisionObject()
    x.id = "Funny Sphere"
    x.header.frame_id = frame_id
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
    time.sleep(1.0)

    # Now publish a mesh and use the db field for the mesh-resource name
    y = CollisionObject()
    y.id = "Funny Mesh"
    y.header.frame_id = frame_id
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
    time.sleep(1.0)

    rospy.spin()
