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
import signal
import subprocess

from simple_script_server import simple_script_server  ## pylint: disable=no-name-in-module
import twist_controller_config as tcc
from dynamic_reconfigure.client import Client

from data_collection import JointStateDataKraken
from data_collection import ObstacleDistanceDataKraken
from data_collection import TwistDataKraken
from data_collection import JointVelocityDataKraken
from data_collection import FrameTrackingDataKraken

# has to be startet with ns param: rosrun cob_twist_controller collect_twist_control_eval_data.py __ns:=arm_right
def init_dyn_recfg():
    cli = tcc.TwistControllerReconfigureClient()
    cli.init()

    cli.set_config_param(tcc.NUM_FILT, False)
    cli.set_config_param(tcc.DAMP_METHOD, tcc.TwistController_MANIPULABILITY)
    cli.set_config_param(tcc.LAMBDA_MAX, 0.1)
    cli.set_config_param(tcc.W_THRESH, 0.05)

#     cli.set_config_param(tcc.DAMP_METHOD, tcc.TwistController_LEAST_SINGULAR_VALUE)
#     cli.set_config_param(tcc.LAMBDA_MAX, 0.2)
#     cli.set_config_param(tcc.EPS_DAMP, 0.2)
    cli.set_config_param(tcc.EPS_TRUNC, 0.001)

    cli.set_config_param(tcc.PRIO_CA, 100)
    cli.set_config_param(tcc.PRIO_JLA, 50)

    cli.set_config_param(tcc.SOLVER, tcc.TwistController_STACK_OF_TASKS)
    cli.set_config_param(tcc.K_H, 1.0)

    cli.set_config_param(tcc.CONSTR_CA, tcc.TwistController_CA)
    cli.set_config_param(tcc.K_H_CA, -2.0)
    cli.set_config_param(tcc.ACTIV_THRESH_CA, 0.1)
    cli.set_config_param(tcc.ACTIV_BUF_CA, 50.0)
    cli.set_config_param(tcc.CRIT_THRESH_CA, 0.025)
    cli.set_config_param(tcc.DAMP_CA, 0.000001)

    cli.set_config_param(tcc.CONSTR_JLA, tcc.TwistController_JLA)
    cli.set_config_param(tcc.K_H_JLA, -1.0)
    cli.set_config_param(tcc.ACTIV_THRESH_JLA, 10.0)
    cli.set_config_param(tcc.ACTIV_BUF_JLA, 300.0)
    cli.set_config_param(tcc.CRIT_THRESH_JLA, 5.0)
    cli.set_config_param(tcc.DAMP_JLA, 0.00001)

    cli.set_config_param(tcc.KIN_EXT, tcc.TwistController_NO_EXTENSION)
    cli.set_config_param(tcc.KEEP_DIR, True)
    cli.set_config_param(tcc.ENF_VEL_LIM, True)
    cli.set_config_param(tcc.ENF_POS_LIM, True)

    cli.update()
    cli.close()

    cli = Client('frame_tracker')
    ft_param = {'cart_min_dist_threshold_lin' : 0.2, 'cart_min_dist_threshold_rot' : 0.2}
    cli.update_configuration(ft_param)
    cli.close()


def init_pos():
    sss = simple_script_server()
    sss.move("arm_right", "home")
    sss.move("arm_right", [[-0.039928546971938594, 0.7617065276699337, 0.01562043859727158, -1.7979678396373568, 0.013899422367821046, 1.0327319085987252, 0.021045294231647915]])


if __name__ == "__main__":
    rospy.init_node("test_careobot_st_jla_ca_torus")

    base_dir = '/home/fxm-mb/bag-files/2015_08_06/'
    if rospy.has_param('~base_dir'):
        base_dir = rospy.get_param('~base_dir')
    else:
        rospy.logwarn('Could not find parameter ~base_dir. Using default base_dir: ' + base_dir)

    if rospy.has_param('chain_tip_link'):
        chain_tip_link = rospy.get_param('chain_tip_link')
    else:
        rospy.logwarn('Could not find parameter chain_tip_link.')
        exit(-1)

    if rospy.has_param('frame_tracker/tracking_frame'):
        tracking_frame = rospy.get_param('frame_tracker/tracking_frame')
    else:
        rospy.logwarn('Could not find parameter frame_tracker/tracking_frame.')
        exit(-2)

    if rospy.has_param('root_frame'):
        root_frame = rospy.get_param('root_frame')
    else:
        rospy.logwarn('Could not find parameter root_frame.')
        exit(-3)

    t = time.localtime()
    launch_time_stamp = time.strftime("%Y%m%d_%H_%M_%S", t)
    command = 'rosbag play ' + base_dir + 'careobot_st_jla_ca.bag'

    data_krakens = [
                    JointStateDataKraken(base_dir + 'careobot_st_jla_ca_joint_state_data_' + launch_time_stamp + '.csv'),
                    ObstacleDistanceDataKraken(base_dir + 'careobot_st_jla_ca_obst_dist_data_' + launch_time_stamp + '.csv'),
                    TwistDataKraken(base_dir + 'careobot_st_jla_ca_twist_data_' + launch_time_stamp + '.csv'),
                    JointVelocityDataKraken(base_dir + 'careobot_st_jla_ca_joint_vel_data_' + launch_time_stamp + '.csv'),
                    FrameTrackingDataKraken(base_dir + 'careobot_st_jla_ca_frame_tracking_data_' + launch_time_stamp + '.csv', root_frame, chain_tip_link, tracking_frame), ]

    init_pos()
    init_dyn_recfg()

    status_open = True
    for data_kraken in data_krakens:
        status_open = status_open and data_kraken.open()
    if status_open:
        rospy.loginfo('Subscribers started for data collection ... \nPress CTRL+C to stop program and write data to the file.')

        traj_marker_command = 'rosrun cob_twist_controller debug_trajectory_marker_node __ns:=' + rospy.get_namespace()
        traj_marker_pid = subprocess.Popen(traj_marker_command, stdin = subprocess.PIPE, shell = True)
        pid = subprocess.Popen(command, stdin = subprocess.PIPE, cwd = base_dir, shell = True)

        # pid.wait()
        time.sleep(1.5)  # give time to switch mode back

        if traj_marker_pid.poll() is not None:
            rospy.logerr("traj_marker_pid returned code. Aborting ...")
            pid.send_signal(signal.SIGINT)
            pid.kill()
            # traj_marker_pid.send_signal(signal.SIGINT)
            traj_marker_pid.send_signal(signal.SIGTERM)
            traj_marker_pid.kill()
            exit()

        time.sleep(0.5)  # give time to switch mode back

        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                rate.sleep()
                if(pid.poll() is not None):
                    rospy.loginfo('Bag file finished stop script.')
                    break

        except (KeyboardInterrupt, SystemExit) as e:
            rospy.loginfo('KeyboardInterrupt / SystemExit: ' + str(e))
            # save data
            for data_kraken in data_krakens:
                data_kraken.writeAllData()
        except:
            rospy.logerr('Else exception.')
        else:
            for data_kraken in data_krakens:
                data_kraken.writeAllData()

        try:
            # pid.send_signal(signal.SIGINT)
            pid.kill()
            pid.send_signal(signal.SIGINT)
        except Exception as e:
            rospy.logerr('Failed to stop rosbag play due to exception: ' + str(e))
        try:
            traj_marker_pid.kill()
            traj_marker_pid.send_signal(signal.SIGINT)
        except Exception as e:
            rospy.logerr('Failed to stop debug_trajectory_marker_node due to exception: ' + str(e))
    else:
        rospy.logerr('Failed to open DataKraken files.')







