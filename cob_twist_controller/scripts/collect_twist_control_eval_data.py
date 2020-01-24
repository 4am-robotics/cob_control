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
            rospy.loginfo('KeyboardInterrupt / SystemExit: ' + str(e))
            # save data
            for data_kraken in data_krakens:
                data_kraken.writeAllData()
        except:
            rospy.logerr('Else exception.')
        else:
            for data_kraken in data_krakens:
                data_kraken.writeAllData()
    else:
        rospy.logerr('Failed to open DataKraken files.')
