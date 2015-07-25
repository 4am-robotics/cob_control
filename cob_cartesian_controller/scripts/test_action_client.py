#! /usr/bin/env python
import rospy
import actionlib

from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal

if __name__ == '__main__':
    rospy.init_node('test_action_client')
    client = actionlib.SimpleActionClient('cartesian_trajectory_action', CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer...")
    client.wait_for_server()
    rospy.logwarn("...done")

    # Fill in the goal here
    goal = CartesianControllerGoal()
    
#    goal.name = "move_lin"
#    goal.move_lin.x = -0.9
#    goal.move_lin.y = 0
#    goal.move_lin.z = 0
#    goal.move_lin.roll = 0
#    goal.move_lin.pitch = 0
#    goal.move_lin.yaw = 0
#    goal.move_lin.rotate_only = False
#
#    goal.move_lin.profile.vel = 0.2
#    goal.move_lin.profile.accl = 0.1
#    goal.move_lin.profile.profile_type = 2 #SINOID
     
    goal.name = "hold"
    goal.hold_time = 5


    # Send the goal
    client.send_goal(goal)
