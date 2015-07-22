#! /usr/bin/env python

import roslib
roslib.load_manifest('cob_cartesian_controller')
import rospy
import actionlib

from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal

if __name__ == '__main__':
    rospy.init_node('CartesianControllerAction_client')
    client = actionlib.SimpleActionClient('/arm_left/cartesian_trajectory_action_', CartesianControllerAction)
    client.wait_for_server()

    goal = CartesianControllerGoal()
    
#    goal.ta.name = "move_lin"
#    goal.ta.move_lin.x = -0.9
#    goal.ta.move_lin.y = 0
#    goal.ta.move_lin.z = 0
#    goal.ta.move_lin.roll = 0
#    goal.ta.move_lin.pitch = 0
#    goal.ta.move_lin.yaw = 0
#    goal.ta.move_lin.vel = 0.2
#    goal.ta.move_lin.accl = 0.1
#    goal.ta.move_lin.profile = "sinoide"
#    goal.ta.move_lin.rotateOnly = False
     
    goal.ta.name = "hold"
    goal.ta.hold_time = 5
    # Fill in the goal here
    client.send_goal(goal)
