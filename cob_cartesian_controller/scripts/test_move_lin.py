#! /usr/bin/env python
import rospy
import actionlib

from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile

if __name__ == '__main__':
    rospy.init_node('test_move_lin')
    client = actionlib.SimpleActionClient('cartesian_trajectory_action', CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer...")
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
    goal.move_lin.rotate_only = False

    goal.move_lin.profile.vel = 0.2
    goal.move_lin.profile.accl = 0.1
    goal.move_lin.profile.profile_type = Profile.SINOID

    print goal

    # Send the goal
    client.send_goal(goal)
    client.wait_for_result()
