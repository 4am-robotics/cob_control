#! /usr/bin/env python
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
