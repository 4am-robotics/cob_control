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
from geometry_msgs.msg import Twist

def twistPub():
  rospy.init_node("test_publisher_twist", anonymous=True)

  pub = rospy.Publisher("twist_controller/command_twist", Twist, queue_size=1)
  twist_msg =  Twist()

  twist_msg.linear.x = 0
  twist_msg.linear.y = -0.05
  twist_msg.linear.z = 0
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = 0

  r = rospy.Rate(50)

  while not rospy.is_shutdown():
    pub.publish(twist_msg)
    r.sleep()

if __name__ == '__main__':
  try:
      twistPub()
  except rospy.ROSInterruptException: pass
