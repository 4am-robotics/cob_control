#! /usr/bin/env python
import time

import rospy
import actionlib

from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile

from simple_script_server.simple_script_server import simple_script_server

import simple_cartesian_interface as sci
import twist_controller_config as tcc

from tf.transformations import *

class MoveLin(object):
    
    def __init__(self, world, profile, rpy=(1.571, -0.0004, -0.0114,)):
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
    
def move_rectangle():
    world = "odom_combined"
    profile = Profile()
    profile.vel = 0.2
    profile.accl = 0.1
    profile.profile_type = Profile.SINOID
    
    ml = MoveLin(world, profile)

    ml.move('prepare to move', [0, -0.9, 1])
    ml.move('Move to upper left point', [-0.4, -0.6, 1], hold_duration=0.3)    
    ml.move('Move to bottom left point', [-0.4, -0.9, 1], hold_duration=0.3) 
    ml.move('Move to bottom right point', [0.4, -0.9, 1], hold_duration=0.3) 
    ml.move('Move to upper right point', [0.4, -0.6, 1])  
    #ml.move('Move to bottom right point(home)', [0, -1, 1], hold_duration=0.3)
   
    
    

def init_dyn_recfg():
    cli = tcc.TwistControllerReconfigureClient()
    cli.init()
    
    cli.set_config_param(tcc.DAMP_METHOD, tcc.TwistController_MANIPULABILITY)
    cli.set_config_param(tcc.LAMBDA_MAX, 0.1)
    cli.set_config_param(tcc.W_THRESH, 0.05)
    cli.set_config_param(tcc.SOLVER, tcc.TwistController_STACK_OF_TASKS)
    cli.set_config_param(tcc.K_H, 1.0)
    
    cli.set_config_param(tcc.CONSTR_CA, tcc.TwistController_CA)
    cli.set_config_param(tcc.K_H_CA, -1.9)
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
    
    time.sleep(1.0)
    
    cli.set_config_param(tcc.K_H_CA, -2.0)
    cli.update()
    
    cli.close()


if __name__ == '__main__':
    rospy.init_node('test1')
    
    action_name = rospy.get_namespace()+'cartesian_trajectory_action'
    client = actionlib.SimpleActionClient(action_name, CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer: %s", action_name)
    client.wait_for_server()
    rospy.logwarn("...done")
    
    init_dyn_recfg()
    
    sss = simple_script_server()
    sss.move("arm_right", "home")
    sss.move("arm_left", "home")
    move_rectangle()

