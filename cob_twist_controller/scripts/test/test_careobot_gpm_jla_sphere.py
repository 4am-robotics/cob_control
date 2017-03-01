#!/usr/bin/env python
"""
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: July, 2015
 *
 * \brief
 *   Simple Python node to collect data from several topics.
 *   Subscription and how data is written to a file is done in data_collection.py module.
 *
"""
import time
import rospy
import subprocess

from simple_script_server.simple_script_server import simple_script_server
import twist_controller_config as tcc
from dynamic_reconfigure.client import Client
from visualization_msgs.msg import InteractiveMarkerFeedback
from sensor_msgs.msg import JointState

from data_collection import JointStateDataKraken
from data_collection import TwistDataKraken
from data_collection import JointVelocityDataKraken
from data_collection import FrameTrackingDataKraken

# has to be startet with ns param: rosrun cob_twist_controller collect_twist_control_eval_data.py __ns:=arm_right
def init_dyn_recfg():
    cli = tcc.TwistControllerReconfigureClient()
    cli.init()
    cli.set_config_param(tcc.CTRL_IF, tcc.TwistController_VELOCITY_INTERFACE)

    cli.set_config_param(tcc.DAMP_METHOD, tcc.TwistController_MANIPULABILITY)
    cli.set_config_param(tcc.LAMBDA_MAX, 0.1)
    cli.set_config_param(tcc.W_THRESH, 0.05)

    cli.set_config_param(tcc.PRIO_CA, 100)
    cli.set_config_param(tcc.PRIO_JLA, 50)

    cli.set_config_param(tcc.SOLVER, tcc.TwistController_GPM)
    cli.set_config_param(tcc.K_H, 1.0)

    cli.set_config_param(tcc.CONSTR_CA, tcc.TwistController_CA_OFF)
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
    cli.set_config_param(tcc.KEEP_DIR, False)
    cli.set_config_param(tcc.ENF_VEL_LIM, False)
    cli.set_config_param(tcc.ENF_POS_LIM, False)

    cli.update()
    cli.close()

    cli = Client('frame_tracker')
    ft_param = {'cart_min_dist_threshold_lin' : 0.2, 'cart_min_dist_threshold_rot' : 0.2}
    cli.update_configuration(ft_param)
    cli.close()
# receives the joint states
def jointStateCallback(self, msg):
    for i in range(0,len(msg.position)):
         self.joint_states[i] = msg.position[i]

# wait until arm goal pose reached
def armWaitUntilGoalPoseReached(goal_state):
        reached = False
        while not rospy.is_shutdown() and reached==False:
            reached = armGoalPoseReached(goal_state)
            rospy.sleep(0.5)
        print "arm goal pose reached"

    # check if arm goal state is reached
def armGoalPoseReached(goal_state):
    eps = 0.01
    for i in range(0,len(goal_state)):
        if abs(joint_states[i]-goal_state[i])>eps:
            return False
    return True

def init_pos(goal_state):
    sss = simple_script_server()
    sss.move("arm_left", goal_state)
    sss.move("arm_left", "home")

def jointStateCallback(msg):
    for i in range(0,len(msg.position)):
        joint_states[i] = msg.position[i]

if __name__ == "__main__":
    rospy.init_node("test_careobot_st_jla_ca_sphere")

    base_dir = '/home/bbrito/bag-files/2016_02_22/'
    if rospy.has_param('~base_dir'):
        base_dir = rospy.get_param('~base_dir')
    else:
        rospy.logwarn('Could not find parameter ~base_dir. Using default base_dir: ' + base_dir)

    action_name = rospy.get_namespace()
    if rospy.has_param(action_name+'chain_tip_link'):
        chain_tip_link = rospy.get_param(action_name+'chain_tip_link')
    else:
        rospy.logwarn('Could not find parameter chain_tip_link.')
        exit(-1)

    if rospy.has_param(action_name+'frame_tracker/target_frame'):
        tracking_frame = rospy.get_param(action_name+'frame_tracker/target_frame')
    else:
        rospy.logwarn('Could not find parameter frame_tracker/tracking_frame.')
        exit(-2)

    if rospy.has_param(action_name+'root_frame'):
        root_frame = rospy.get_param(action_name+'root_frame')
    else:
        rospy.logwarn('Could not find parameter root_frame.')
        exit(-3)

    t = time.localtime()
    launch_time_stamp = time.strftime("%Y%m%d_%H_%M_%S", t)
    rospy.Subscriber("/arm_left/joint_states", JointState, jointStateCallback)
    command = 'rosbag play -r 0.5 ' + base_dir + '/experiment1/experiment1.bag'
    # command = 'rosbag play -u 10 ' + base_dir + 'careobot_st_jla_ca_sphere.bag'

    data_krakens = [
                    JointStateDataKraken(base_dir + 'joint_state_data_' + launch_time_stamp + 'GPM.csv'),
                    TwistDataKraken(base_dir + 'twist_data_' + launch_time_stamp + 'GPM.csv'),
                    JointVelocityDataKraken(base_dir + 'joint_vel_data_' + launch_time_stamp + 'GPM.csv'),
                    FrameTrackingDataKraken(base_dir + 'frame_tracking_data_' + launch_time_stamp + 'GPM.csv', root_frame, chain_tip_link, tracking_frame), ]

    joint_states=[1., 0., 0., 0., 0., 0.,0.]
    init_pos(joint_states)
    armWaitUntilGoalPoseReached(joint_states)
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
            pid.send_signal(subprocess.signal.SIGINT)
            pid.kill()
            # traj_marker_pid.send_signal(subprocess.signal.SIGINT)
            traj_marker_pid.send_signal(subprocess.signal.CTRL_C_EVENT)
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
        except rospy.ROSInterruptException as e:
            rospy.logwarn('ROSInterruptException: ' + str(e))
        except:
            rospy.logerr('Else exception.')
        else:
            for data_kraken in data_krakens:
                data_kraken.writeAllData()

        try:
            # pid.send_signal(subprocess.signal.SIGINT)
            pid.kill()
            pid.send_signal(subprocess.signal.SIGINT)
        except Exception as e:
            rospy.logerr('Failed to stop rosbag play due to exception: ' + str(e))
        try:
            traj_marker_pid.kill()
            traj_marker_pid.send_signal(subprocess.signal.SIGINT)
        except Exception as e:
            rospy.logerr('Failed to stop debug_trajectory_marker_node due to exception: ' + str(e))
    else:
        rospy.logerr('Failed to open DataKraken files.')







