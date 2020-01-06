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
from geometry_msgs.msg import TwistStamped

def twistPub():
  rospy.init_node("test_publisher_twist_series", anonymous=True)

  pub = rospy.Publisher("twist_controller/command_twist_stamped", TwistStamped, queue_size=1)

  rospy.sleep(1.0)

  twist_msg =  TwistStamped()
  twist_msg.header.stamp = rospy.Time.now()
  twist_msg.header.frame_id = "arm_left_base_link"
  twist_msg.twist.linear.x = 0
  twist_msg.twist.linear.y = 0
  twist_msg.twist.linear.z = 0
  twist_msg.twist.angular.x = 0
  twist_msg.twist.angular.y = 0
  twist_msg.twist.angular.z = 0

  rate = 50
  r = rospy.Rate(rate)

  for i in range(0, 5*rate, 1):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_left_base_link"
    twist_msg.twist.linear.x = 0
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = -0.02
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()

  #pause
  for i in range(0, 1*rate, 1):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_left_base_link"
    twist_msg.twist.linear.x = 0
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = 0
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()

  for i in range(0, 5*rate, 1):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_left_base_link"
    twist_msg.twist.linear.x = 0
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = 0.02
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()

  #pause
  for i in range(0, 1*rate, 1):
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.frame_id = "arm_left_base_link"
    twist_msg.twist.linear.x = 0
    twist_msg.twist.linear.y = 0
    twist_msg.twist.linear.z = 0
    twist_msg.twist.angular.x = 0
    twist_msg.twist.angular.y = 0
    twist_msg.twist.angular.z = 0
    pub.publish(twist_msg)
    r.sleep()

  print("done")

if __name__ == '__main__':
  try:
      twistPub()
  except rospy.ROSInterruptException: pass

