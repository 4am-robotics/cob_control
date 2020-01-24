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


import rospy
import threading
import actionlib

import tf
from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile


def move_lin(pose_goal, frame_id, profile):
    action_name = rospy.get_namespace()+'cartesian_trajectory_action'
    client = actionlib.SimpleActionClient(action_name, CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer: %s", action_name)
    success = client.wait_for_server(rospy.Duration(2.0))
    if(not success):
        return (success, "ActionServer not available within timeout")

    goal = CartesianControllerGoal()
    goal.move_type = CartesianControllerGoal.LIN
    goal.move_lin.pose_goal = pose_goal
    goal.move_lin.frame_id = frame_id
    goal.profile = profile
    # print goal

    client.send_goal(goal)
    print("goal sent")
    state = client.get_state()
    # print state
    client.wait_for_result()
    print("result received")
    result = client.get_result()
    return (result.success, result.message)

def move_circ(pose_center, frame_id, start_angle, end_angle, radius, profile):
    action_name = rospy.get_namespace()+'cartesian_trajectory_action'
    client = actionlib.SimpleActionClient(action_name, CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer: %s", action_name)
    success = client.wait_for_server(rospy.Duration(2.0))
    if(not success):
        return (success, "ActionServer not available within timeout")

    goal = CartesianControllerGoal()
    goal.move_type = CartesianControllerGoal.CIRC
    goal.move_circ.pose_center = pose_center
    goal.move_circ.frame_id = frame_id
    goal.move_circ.start_angle = start_angle
    goal.move_circ.end_angle = end_angle
    goal.move_circ.radius = radius
    goal.profile = profile
    # print goal

    client.send_goal(goal)
    print("goal sent")
    state = client.get_state()
    # print state
    client.wait_for_result()
    print("result received")
    result = client.get_result()
    return (result.success, result.message)



####################################
#  helpers
####################################


'''
Generates a geometry_msgs.Pose from (x,y,z) and (r,p,y)
'''
def gen_pose(pos=[0,0,0], rpy=[0,0,0]):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = pos
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = tf.transformations.quaternion_from_euler(*rpy)

    return pose
