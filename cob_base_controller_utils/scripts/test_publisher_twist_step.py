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
import random
from geometry_msgs.msg import Twist


def velPub():
  rospy.init_node("test_publisher_twist", anonymous=True)

  pub_vel = rospy.Publisher("command", Twist, queue_size=1)
  rospy.sleep(1.0)

  freq = 10.0
  r = rospy.Rate(freq)
  
  vel_msg = Twist()

  while not rospy.is_shutdown():
    time = rospy.Time.now()
    since_start = 0.0
    vel = random.uniform(-1.5, 1.5)
    while since_start < random.uniform(2.0, 5.0):
      since_start = (rospy.Time.now() - time).to_sec()

      vel_msg.angular.z = vel
      pub_vel.publish(vel_msg)
      r.sleep()

if __name__ == '__main__':
  try:
      velPub()
  except rospy.ROSInterruptException: pass
