#! /usr/bin/env python

import roslib
roslib.load_manifest('cob_cartesian_controller')
import rospy
import actionlib

from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile

'''
MANIPULABILITY: 0.1 lambda_max, 0.005 w_threshold
DYN_TASK_READJ
mu = -0.1
CA mit k_H_CA = -10.0

JLA_INEQ mit k_H_JLA = 1.0

START FROM HOME_POSITION


'''

def move_test():
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = -0.2
    goal.move_lin.y = 0
    goal.move_lin.z = -0.2
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()

   
#     goal = CartesianControllerGoal()
#     goal.name = "move_lin"
#     goal.move_lin.x = 0.0
#     goal.move_lin.y = 0.0
#     goal.move_lin.z = -0.2
#     goal.move_lin.roll = 0.0
#     goal.move_lin.pitch = 0.0
#     goal.move_lin.yaw = 0.0
#     goal.move_lin.profile.vel = 0.1
#     goal.move_lin.profile.accl = 0.2
#     goal.move_lin.profile.profile_type = Profile.SINOID
#     goal.move_lin.rotate_only = False
#     client.send_goal(goal)
#     client.wait_for_result()
    
    
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.2
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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 0.5
    client.send_goal(goal)
    client.wait_for_result()
    
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.65
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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 0.5
    client.send_goal(goal)
    client.wait_for_result()
    
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.3
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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 0.5
    client.send_goal(goal)
    client.wait_for_result()
    
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = -0.25
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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 0.5
    client.send_goal(goal)
    client.wait_for_result()
    
    # Try to grap down wards in the torus
    rospy.loginfo("Try to move down behind torus")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.0
    goal.move_lin.z = -0.1
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = -45.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("Result: " + str(result))
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 0.5
    client.send_goal(goal)
    client.wait_for_result()
    
    # Try to grap up wards in the torus
    rospy.loginfo("Try to move up behind torus")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.0
    goal.move_lin.z = 0.4
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 45.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("Result: " + str(result))
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 0.5
    client.send_goal(goal)
    client.wait_for_result()
    
    rospy.loginfo("Try to move down again")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.0
    goal.move_lin.z = -0.1
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("Result: " + str(result))
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 0.5
    client.send_goal(goal)
    client.wait_for_result()
    
    rospy.loginfo("Try to move back")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = -0.6
    goal.move_lin.y = 0.0
    goal.move_lin.z = 0.0
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()

#     goal = CartesianControllerGoal()
#     goal.name = "hold"
#     goal.hold_time = 0.5
#     client.send_goal(goal)
#     client.wait_for_result()
#     
#     goal = CartesianControllerGoal()
#     goal.name = "move_lin"
#     goal.move_lin.x = 0.0
#     goal.move_lin.y = 0.3
#     goal.move_lin.z = 0.0
#     goal.move_lin.roll = 0.0
#     goal.move_lin.pitch = 0.0
#     goal.move_lin.yaw = 0.0
#     goal.move_lin.profile.vel = 0.1
#     goal.move_lin.profile.accl = 0.2
#     goal.move_lin.profile.profile_type = Profile.SINOID
#     goal.move_lin.rotate_only = False
#     client.send_goal(goal)
#     client.wait_for_result()
    
    


def move_lin_x(client, length):
#     goal = CartesianControllerGoal()
#     goal.name = "move_lin"
#     goal.move_lin.x = length
#     goal.move_lin.y = 0
#     goal.move_lin.z = 0
#     goal.move_lin.roll = 0.0
#     goal.move_lin.pitch = 0.0
#     goal.move_lin.yaw = 0.0
#     goal.move_lin.profile.vel = 0.1
#     goal.move_lin.profile.accl = 0.2
#     goal.move_lin.profile.profile_type = Profile.SINOID
#     goal.move_lin.rotate_only = False
#     client.send_goal(goal)
#     client.wait_for_result()
# 
#     print("Finished move_lin")

#     goal = CartesianControllerGoal()
#     goal.name = "hold"
#     goal.hold_time = 2
#     client.send_goal(goal)
#     client.wait_for_result()
#     
#     print("Finished hold")
# 
    # Attention: Changes the endeffector position with reference to the world_frame, but the orientation will be changed relatively to the current orientation. 
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.5
    goal.move_lin.y = 0
    goal.move_lin.z = 0
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 45.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()



def move_santa(client):
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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 4
    client.send_goal(goal)
    client.wait_for_result()
    
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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 4
    client.send_goal(goal)
    client.wait_for_result()
    
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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 2
    client.send_goal(goal)
    client.wait_for_result()
    
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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 2
    client.send_goal(goal)
    client.wait_for_result()
    
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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 2
    client.send_goal(goal)
    client.wait_for_result()

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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 2
    client.send_goal(goal)
    client.wait_for_result()
    
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
    
    goal = CartesianControllerGoal()
    goal.name = "hold"
    goal.hold_time = 2
    client.send_goal(goal)
    client.wait_for_result()
    
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

#    goal.name = "move_lin"
#    goal.move_lin.x = -0.9
#    goal.move_lin.y = 0
#    goal.move_lin.z = 0
#    goal.move_lin.roll = 0
#    goal.move_lin.pitch = 0
#    goal.move_lin.yaw = 0
#    goal.move_lin.rotate_only = False
#
#    goal.move_lin.profile.profile.vel = 0.2
#    goal.move_lin.profile.profile.accl = 0.1
#    goal.move_lin.profile.profile_type = 2 #SINOID

if __name__ == '__main__':
    rospy.init_node('CartesianControllerAction_client')
    
    client = actionlib.SimpleActionClient('cartesian_trajectory_action_', CartesianControllerAction)
    client.wait_for_server()
    #move_santa(client)
    #move_lin_x(client, -0.5)
    
    move_test()
    
