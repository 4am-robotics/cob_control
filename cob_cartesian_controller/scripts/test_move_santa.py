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

    # Step 1
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = -1.0
    goal.move_lin.y = 0
    goal.move_lin.z = 0
    goal.move_lin.roll = 45.0
    goal.move_lin.pitch = 0
    goal.move_lin.yaw = -25.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()

    rospy.sleep(4.0)

    # Step 2
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = -0.5
    goal.move_lin.y = 0.5
    goal.move_lin.z = 0
    goal.move_lin.roll = -45.0
    goal.move_lin.pitch = 0
    goal.move_lin.yaw = -30.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()

    rospy.sleep(4.0)

    # Step 3
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.5
    goal.move_lin.y = 0.5
    goal.move_lin.z = 0
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()

    rospy.sleep(2.0)

    # Step 4
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 1.0
    goal.move_lin.y = 0.0
    goal.move_lin.z = 0
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()

    rospy.sleep(2.0)

    # Step 5
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = -1.0
    goal.move_lin.z = 0
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()

    rospy.sleep(2.0)

    # Step 6
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = -1.0
    goal.move_lin.y = 1.0
    goal.move_lin.z = 0
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()

    rospy.sleep(2.0)

    # Step 7
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = -1.0
    goal.move_lin.z = 0
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()

    rospy.sleep(2.0)

    # Step 8
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 1.0
    goal.move_lin.y = 1.0
    goal.move_lin.z = 0
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()
