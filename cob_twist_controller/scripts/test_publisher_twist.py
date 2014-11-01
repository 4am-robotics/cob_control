#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist

 
def twistPub():
  rospy.init_node("test_publisher_twist", anonymous=True)
  
  pub = rospy.Publisher("/arm_controller/command_twist", Twist, queue_size=1)
  twist_msg =  Twist()

  twist_msg.linear.x = 0
  twist_msg.linear.y = 0
  twist_msg.linear.z = 0
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = 0
   
  r = rospy.Rate(10)
  
  while not rospy.is_shutdown():
    pub.publish(twist_msg)
    r.sleep()

if __name__ == '__main__':
  try:
      twistPub()
  except rospy.ROSInterruptException: pass
