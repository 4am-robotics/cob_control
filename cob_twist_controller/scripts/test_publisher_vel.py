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


import rospy
import math
from std_msgs.msg import Float64MultiArray

def velPub():
  rospy.init_node("test_publisher_vel", anonymous=True)

  pub_vel = rospy.Publisher("joint_group_velocity_controller/command", Float64MultiArray, queue_size=1)
  rospy.sleep(1.0)

  freq = 100.0
  r = rospy.Rate(freq)
  
  joint_idx = 2

  a = 0.1#2.0
  b = 0.1 * (2.0*math.pi/freq)
  c = 0.0 #math.pi
  d = 0.0
  i = 0.0
  
  vel_msg = Float64MultiArray()
  vel_msg.data = [0.0] * 7

  while not rospy.is_shutdown():
    vel = a*math.sin(b*i+c) + d
    #print("vel: ", vel)
    vel_msg.data[joint_idx] = vel
    pub_vel.publish(vel_msg)
    i += 1.0
    r.sleep()

if __name__ == '__main__':
  try:
      velPub()
  except rospy.ROSInterruptException: pass
