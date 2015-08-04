#! /usr/bin/env python

import roslib
roslib.load_manifest('cob_cartesian_controller')
import rospy
import actionlib
import time

from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile

from simple_script_server.simple_script_server import simple_script_server 

from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import Profile

import simple_cartesian_interface as sci
import twist_controller_config as tcc

from tf.transformations import *
'''
Test precondition!

MANIPULABILITY: 0.1 lambda_max, 0.005 w_threshold
DYN_TASK_READJ
CA mit k_H_CA = -20.0

JLA mit k_H_JLA = -1.0

START FROM HOME_POSITION

Torus with Radius 0.25 should be placed here (world frame / e.g. odom combined):  
    pose.position.x = 0.25
    pose.position.y = -0.60
    pose.position.z = 0.95

'''

def move_test():
    # Move back down
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = -0.2
    goal.move_lin.y = 0
    goal.move_lin.z = -0.07
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.1
    goal.move_lin.profile.accl = 0.2
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()

    time.sleep(0.1)    
    
    # Move into the direction of the torso
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.35
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
    
    time.sleep(0.1)
    
    rospy.loginfo("Try to move through")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.8
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
    
    time.sleep(0.5)
    
    rospy.loginfo("Try to move through")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.1
    goal.move_lin.y = 0.0
    goal.move_lin.z = 0
    goal.move_lin.roll = 0.0
    goal.move_lin.pitch = 0.0
    goal.move_lin.yaw = 0.0
    goal.move_lin.profile.vel = 0.05
    goal.move_lin.profile.accl = 0.1
    goal.move_lin.profile.profile_type = Profile.SINOID
    goal.move_lin.rotate_only = False
    client.send_goal(goal)
    client.wait_for_result()
    
    time.sleep(0.5)
    
       
    rospy.loginfo("Try to move to robot")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.15
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
    
    time.sleep(0.1)
    
    rospy.loginfo("Try to move away from robot")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = -0.3
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
    
    time.sleep(0.1)
    
    # Try to grap down wards in the torus
    rospy.loginfo("Try to move down behind torus")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.0
    goal.move_lin.z = -0.15
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
    
    time.sleep(0.3)
    
    # Try to grap up wards in the torus
    rospy.loginfo("Try to move up behind torus")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.0
    goal.move_lin.z = 0.35
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
    
    time.sleep(0.3)
    
    rospy.loginfo("Try to move down again")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.0
    goal.move_lin.z = -0.15
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
    
    time.sleep(0.3)
    
    rospy.loginfo("Try to move to robot")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = 0.0
    goal.move_lin.y = 0.2
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
    
    time.sleep(0.3)
    
    rospy.loginfo("1) Try to move back")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = -0.4
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
    
    time.sleep(0.3)
    
    rospy.loginfo("2) Try to move back")
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.x = -0.2
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
    
    
class MoveLin(object):
    
    def __init__(self, world, profile, rpy=(1.571, -0.0004, -0.0114,)): # keep ee orientation as started
        self._world = world
        self._profile = profile
        self._rpy = rpy
        
    def move(self, title, pos=[0.0, 0.0, 0.0], rpy=None, hold_duration=None):
        rospy.loginfo(title)
        if rpy is not None: 
            self._rpy = rpy
        pose = sci.gen_pose(pos, self._rpy)
        success = sci.move_lin(pose, self._world, self._profile)
        if hold_duration is not None: 
            time.sleep(hold_duration)
        return success
    




def move_around_torus():
    world = "odom_combined"
    profile = Profile()
    profile.vel = 0.2
    profile.accl = 0.1
    profile.profile_type = Profile.SINOID
    
    ml = MoveLin(world, profile)

    ml.move('Move out of singularity', [-0.094, -0.987, 0.93])
    ml.move('Move to center of torus', [-0.09, -0.607, 0.93])
    ml.move('Move through torus', [0.558, -0.614, 0.93])
    ml.move('Move to torso', [0.56, -0.25, 0.93], hold_duration=0.3)    
    ml.move('Move away from torso', [0.56, -0.69, 0.93], hold_duration=0.3)    
    ml.move('1st) Move to center of torus (front)', [0.45, -0.66, 0.94], hold_duration=0.3)
    ml.move('2nd time for safety. Move to center of torus (front)', [0.45, -0.66, 0.94], hold_duration=0.3)
    ml.move('Move through torus (behind)', [-0.09, -0.607, 0.93], hold_duration=0.3)
    ml.move('Move through torus again (front)', [0.558, -0.607, 0.93], hold_duration=0.3)
    ml.move('Move up', [0.558, -0.607, 1.4], hold_duration=0.3)
    ml.move('Move down to center of torus', [0.558, -0.607, 0.93], hold_duration=0.3)   
    ml.move('1st) Move down', [0.558, -0.607, 0.5], hold_duration=0.4) # leave some time for self-reconfig
    ml.move('2nd) Move down', [0.558, -0.607, 0.5], hold_duration=0.4) # leave some time for self-reconfig
    ml.move('3rd) Move down', [0.558, -0.607, 0.5], hold_duration=0.4) # leave some time for self-reconfig
    # ml.move('Move up to center of torus', [0.558, -0.607, 0.93], hold_duration=0.3)
    # ml.move('Move through torus again (behind)', [-0.09, -0.607, 0.93], hold_duration=0.3)
    

def init_dyn_recfg():
    cli = tcc.TwistControllerReconfigureClient()
    cli.init()
    
    cli.set_config_param(tcc.DAMP_METHOD, tcc.TwistController_MANIPULABILITY)
    cli.set_config_param(tcc.LAMBDA_MAX, 0.1)
    cli.set_config_param(tcc.W_THRESH, 0.005)
    cli.set_config_param(tcc.SOLVER, tcc.TwistController_DYN_TASKS_READJ)
    cli.set_config_param(tcc.K_H, 1.0)
    
    cli.set_config_param(tcc.CONSTR_CA, tcc.TwistController_CA)
    cli.set_config_param(tcc.K_H_CA, -20.0)
    cli.set_config_param(tcc.DAMP_CA, 0.000001)
    cli.set_config_param(tcc.ACTIV_THRESH_CA, 0.1)
    
    cli.set_config_param(tcc.CONSTR_JLA, tcc.TwistController_JLA)
    cli.set_config_param(tcc.K_H_JLA, -1.0)
    cli.set_config_param(tcc.DAMP_JLA, 0.00001)
    cli.set_config_param(tcc.ACTIV_THRESH_JLA, 10.0)
    
    cli.set_config_param(tcc.KIN_EXT, tcc.TwistController_NO_EXTENSION)
    cli.set_config_param(tcc.KEEP_DIR, True)
    cli.set_config_param(tcc.ENF_VEL_LIM, True)
    cli.set_config_param(tcc.ENF_POS_LIM, True)
    
    cli.update()
    cli.close()


if __name__ == '__main__':
    rospy.init_node('test_move_around_torus')
    
    client = actionlib.SimpleActionClient('cartesian_trajectory_action', CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer...")
    client.wait_for_server()
    rospy.logwarn("...done")  
    
    init_dyn_recfg()
    
    sss = simple_script_server()
    sss.move("arm_right", "home")
    
    move_around_torus()
    