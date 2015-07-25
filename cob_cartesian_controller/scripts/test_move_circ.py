#! /usr/bin/env python
import rospy
import actionlib

from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile

if __name__ == '__main__':
    rospy.init_node('test_move_circ')
    client = actionlib.SimpleActionClient('cartesian_trajectory_action', CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer...")
    client.wait_for_server()
    rospy.logwarn("...done")

    # Fill in the goal here
    goal = CartesianControllerGoal()

    goal.name = "move_circ"
    goal.move_circ.x_center = 0.0
    goal.move_circ.y_center = 0.7
    goal.move_circ.z_center = 1.0
    goal.move_circ.roll_center = 0
    goal.move_circ.pitch_center = 0
    goal.move_circ.yaw_center = 0
    goal.move_circ.rotate_only = False

    goal.move_circ.start_angle = 0
    goal.move_circ.end_angle = 90
    goal.move_circ.radius = 0.3

    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID

    # Send the goal
    client.send_goal(goal)
    client.wait_for_result()
