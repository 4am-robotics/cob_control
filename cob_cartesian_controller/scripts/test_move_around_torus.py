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


import time

import rospy
import actionlib

from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile

from simple_script_server import simple_script_server  ## pylint: disable=no-name-in-module

import simple_cartesian_interface as sci
import twist_controller_config as tcc

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
    ml.move('Move up to center of torus', [0.558, -0.607, 0.93], hold_duration=0.3)
    ml.move('Move through torus again (behind)', [-0.09, -0.607, 0.93], hold_duration=0.3)
    

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
    rospy.init_node('test_move_around_torus')
    
    action_name = rospy.get_namespace()+'cartesian_trajectory_action'
    client = actionlib.SimpleActionClient(action_name, CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer: %s", action_name)
    client.wait_for_server()
    rospy.logwarn("...done")
    
    init_dyn_recfg()
    
    sss = simple_script_server()
    sss.move("arm_right", "home")
    
    move_around_torus()
