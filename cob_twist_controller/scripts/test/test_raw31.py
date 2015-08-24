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

from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import LoadController

from simple_script_server.simple_script_server import simple_script_server
import twist_controller_config as tcc
from dynamic_reconfigure.client import Client

from data_collection import JointStateDataKraken
from data_collection import ObstacleDistanceDataKraken
from data_collection import TwistDataKraken
from data_collection import JointVelocityDataKraken
from data_collection import FrameTrackingDataKraken
from data_collection import OdometryDataKraken

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


pub = rospy.Publisher('/arm/joint_trajectory_controller/command', JointTrajectory, queue_size = 10)

# has to be startet with ns param: rosrun cob_twist_controller collect_twist_control_eval_data.py __ns:=arm_right
def init_dyn_recfg():
    cli = tcc.TwistControllerReconfigureClient()
    cli.init()
    cli.set_config_param(tcc.HW_IF_TYPE, tcc.TwistController_VELOCITY_INTERFACE)

    cli.set_config_param(tcc.NUM_FILT, False)
    cli.set_config_param(tcc.DAMP_METHOD, tcc.TwistController_MANIPULABILITY)
    cli.set_config_param(tcc.LAMBDA_MAX, 0.5)
    cli.set_config_param(tcc.W_THRESH, 0.05)
    cli.set_config_param(tcc.EPS_DAMP, 0.03)
    cli.set_config_param(tcc.EPS_TRUNC, 0.001)

    cli.set_config_param(tcc.PRIO_CA, 100)
    cli.set_config_param(tcc.PRIO_JLA, 50)

    cli.set_config_param(tcc.SOLVER, tcc.TwistController_STACK_OF_TASKS)
    cli.set_config_param(tcc.K_H, 1.0)

    cli.set_config_param(tcc.CONSTR_CA, tcc.TwistController_CA)
    cli.set_config_param(tcc.K_H_CA, -1.0)
    cli.set_config_param(tcc.ACTIV_THRESH_CA, 0.1)
    cli.set_config_param(tcc.ACTIV_BUF_CA, 25.0)
    cli.set_config_param(tcc.CRIT_THRESH_CA, 0.05)
    cli.set_config_param(tcc.DAMP_CA, 0.000001)

    cli.set_config_param(tcc.CONSTR_JLA, tcc.TwistController_JLA)
    cli.set_config_param(tcc.K_H_JLA, -0.1)
    cli.set_config_param(tcc.ACTIV_THRESH_JLA, 10.0)
    cli.set_config_param(tcc.ACTIV_BUF_JLA, 300.0)
    cli.set_config_param(tcc.CRIT_THRESH_JLA, 5.0)
    cli.set_config_param(tcc.DAMP_JLA, 0.00001)

    cli.set_config_param(tcc.KIN_EXT, tcc.TwistController_NO_EXTENSION)
    cli.set_config_param(tcc.KEEP_DIR, True)
    cli.set_config_param(tcc.ENF_VEL_LIM, True)
    cli.set_config_param(tcc.ENF_POS_LIM, False)
    cli.set_config_param(tcc.TOL, 0.01)  # To show limits are violate -> no tolerance

    cli.update()
    cli.close()

    cli = Client('frame_tracker')
    ft_param = {'cart_min_dist_threshold_lin' : 5.0, 'cart_min_dist_threshold_rot' : 5.0}
    cli.update_configuration(ft_param)
    cli.close()


def change_dyn_recfg_for_jla(solver_type):
    cli = tcc.TwistControllerReconfigureClient()
    cli.init()

    cli.set_config_param(tcc.CONSTR_CA, tcc.TwistController_CA_OFF)  # switch CA off, now only JLA interesting!

    cli.set_config_param(tcc.SOLVER, solver_type)

    cli.set_config_param(tcc.CONSTR_JLA, tcc.TwistController_JLA)
    cli.set_config_param(tcc.K_H_JLA, -0.1)
    cli.set_config_param(tcc.ACTIV_THRESH_JLA, 10.0)
    cli.set_config_param(tcc.ACTIV_BUF_JLA, 300.0)
    cli.set_config_param(tcc.CRIT_THRESH_JLA, 5.0)
    cli.set_config_param(tcc.DAMP_JLA, 0.00001)

    cli.set_config_param(tcc.KIN_EXT, tcc.TwistController_NO_EXTENSION)
    cli.set_config_param(tcc.KEEP_DIR, True)
    cli.set_config_param(tcc.ENF_VEL_LIM, True)
    cli.set_config_param(tcc.ENF_POS_LIM, False)
    cli.set_config_param(tcc.TOL, 0.01)  # To show limits are violate -> no tolerance
    cli.update()

    time.sleep(0.5)
    print(cli.get_configuration(5.0))
    cli.close()



def init_pos():
    # Simple script server does not work for raw3-1 therefore using trajectory controller directly

    switch_controller = rospy.ServiceProxy('/arm/controller_manager/switch_controller', SwitchController)
    print(switch_controller(['joint_trajectory_controller', ], None, 1))  # switch on

    # Limits arm_elbow_joint: lower=\"-3.14159265\" upper=\"3.14159265\
    # Limits arm_shoulder_lift_joint: lower=\"-3.14159265\" upper=\"3.14159265\"
    # Limits arm_shoulder_pan_joint: lower=\"-3.14159265\" upper=\"3.14159265\"
    # Limits arm_wrist_1_joint: lower=\"-3.14159265\" upper=\"3.14159265\"\
    # Limits arm_wrist_2_joint: lower=\"-3.14159265\" upper=\"3.14159265\"
    # Limits arm_wrist_3_joint: lower=\"-3.14159265\" upper=\"3.14159265\"

    jnt_traj = JointTrajectory()
    jnt_traj.joint_names = ['arm_elbow_joint', 'arm_shoulder_lift_joint', 'arm_shoulder_pan_joint', 'arm_wrist_1_joint', 'arm_wrist_2_joint', 'arm_wrist_3_joint']

    jtp = JointTrajectoryPoint()
    jtp.positions = [1.69, -1.04, -1.38, -0.67, -0.69, 0.04]
    jtp.time_from_start.secs = 1.0
    jnt_traj.points = [jtp]

    time.sleep(1)
    pub.publish(jnt_traj)

    time.sleep(1)


if __name__ == "__main__":
    rospy.init_node("test_raw31_st_jla_ca_torus")

    base_dir = '/home/fxm-mb/bag-files/raw31_experiments/'
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
    command = 'rosbag play ' + base_dir + 'raw31_sca.bag'

    data_krakens = [
                    JointStateDataKraken(base_dir + 'raw31_sca_joint_state_data_' + launch_time_stamp + '.csv'),
                    ObstacleDistanceDataKraken(base_dir + 'raw31_sca_obst_dist_data_' + launch_time_stamp + '.csv'),
                    TwistDataKraken(base_dir + 'raw31_sca_twist_controller_commanded_twist_data_' + launch_time_stamp + '.csv'),
                    JointVelocityDataKraken(base_dir + 'raw31_sca_joint_vel_data_' + launch_time_stamp + '.csv'),
                    FrameTrackingDataKraken(base_dir + 'raw31_sca_frame_tracking_data_' + launch_time_stamp + '.csv', root_frame, chain_tip_link, tracking_frame)
                    ]


    init_pos()
    init_dyn_recfg()

    status_open = True
    for data_kraken in data_krakens:
        status_open = status_open and data_kraken.open()
    if status_open:
        rospy.loginfo('Subscribers started for data collection ... \nPress CTRL+C to stop program and write data to the file.')

        pid = subprocess.Popen(command, stdin = subprocess.PIPE, cwd = base_dir, shell = True)
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
    else:
        rospy.logerr('Failed to open DataKraken files.')

# #############################################################################################################################
# NOW JLA
# MOVE AROUND Z-Axis with 0.3 rad/sec until violated then back with -0.3 rad/sec
# #############################################################################################################################

    for solver_type in (tcc.TwistController_STACK_OF_TASKS, tcc.TwistController_WLN,):
        time.sleep(2.0)

        if solver_type == tcc.TwistController_STACK_OF_TASKS:
            abbr_solver = 'sot'
        elif solver_type == tcc.TwistController_WLN:
            abbr_solver = 'wln'


        t = time.localtime()
        launch_time_stamp = time.strftime("%Y%m%d_%H_%M_%S", t)
        command = 'rosbag play ' + base_dir + 'raw31_jla.bag'



        data_krakens = [
                        JointStateDataKraken(base_dir + 'raw31_' + abbr_solver + '_jla_joint_state_data_' + launch_time_stamp + '.csv'),
                        TwistDataKraken(base_dir + 'raw31_' + abbr_solver + '_jla_twist_controller_commanded_twist_data_' + launch_time_stamp + '.csv'),
                        ]

        init_pos()
        change_dyn_recfg_for_jla(solver_type)


        status_open = True
        for data_kraken in data_krakens:
            status_open = status_open and data_kraken.open()
        if status_open:
            rospy.loginfo('Subscribers started for data collection ... \nPress CTRL+C to stop program and write data to the file.')

            pid = subprocess.Popen(command, stdin = subprocess.PIPE, cwd = base_dir, shell = True)
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
        else:
            rospy.logerr('Failed to open DataKraken files.')





