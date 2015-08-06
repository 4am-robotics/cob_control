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

from data_collection import JointStateDataKraken
from data_collection import ObstacleDistanceDataKraken
from data_collection import TwistDataKraken
from data_collection import JointVelocityDataKraken

# has to be startet with ns param: rosrun cob_twist_controller collect_twist_control_eval_data.py __ns:=arm_right
if __name__=="__main__":
    rospy.init_node("collect_twist_control_eval_data")
       
    base_dir = '/home/fxm-mb/Scripts/Tests/FinalTests/'
    if rospy.has_param('~base_dir'):
        base_dir = rospy.get_param('~base_dir')
    else:
        rospy.logwarn('Could not find parameter ~base_dir. Using default base_dir: ' + base_dir)
      
    data_krakens = [JointStateDataKraken(base_dir + 'joint_state_data.csv'), 
                    ObstacleDistanceDataKraken(base_dir + 'obst_dist_data.csv'),
                    TwistDataKraken(base_dir + 'twist_data.csv'), 
                    JointVelocityDataKraken(base_dir + 'joint_vel_data.csv'), ]    
    
    status_open = True
    for data_kraken in data_krakens: 
        status_open = status_open and data_kraken.open()
    if status_open: 
        rospy.loginfo('Subscribers started for data collection ... \nPress CTRL+C to stop program and write data to the file.')          
        rate = rospy.Rate(10)
        try: 
            while not rospy.is_shutdown():
                rate.sleep()
        except (KeyboardInterrupt, SystemExit) as e:
            print('KeyboardInterrupt / SystemExit: ' + str(e))
            # save data
            for data_kraken in data_krakens: 
                data_kraken.writeAllData()
        except rospy.ROSInterruptException as e:
            print('ROSInterruptException: ' + str(e))
        except: 
            print('Else exception.')
    else: 
        rospy.logerr('Failed to open DataKraken files.')
