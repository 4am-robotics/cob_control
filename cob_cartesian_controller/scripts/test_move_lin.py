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
import actionlib

from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile

if __name__ == '__main__':
    rospy.init_node('test_move_lin')
    action_name = rospy.get_namespace()+'cartesian_trajectory_action'
    client = actionlib.SimpleActionClient(action_name, CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer: %s", action_name)
    client.wait_for_server()
    rospy.logwarn("...done")

    # Fill in the goal here
    goal = CartesianControllerGoal()

    goal.move_type = CartesianControllerGoal.LIN
    goal.move_lin.pose_goal.position.x = -0.9
    goal.move_lin.pose_goal.position.y = 0.0
    goal.move_lin.pose_goal.position.z = 0.0
    goal.move_lin.pose_goal.orientation.x = 0.0
    goal.move_lin.pose_goal.orientation.y = 0.0
    goal.move_lin.pose_goal.orientation.z = 0.0
    goal.move_lin.pose_goal.orientation.w = 1.0
    goal.move_lin.frame_id = 'world'

    goal.profile.vel = 0.2
    goal.profile.accl = 0.1
    goal.profile.profile_type = Profile.SINOID

    #print goal

    # Send the goal
    client.send_goal(goal)
    client.wait_for_result()
