#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import TwistStamped

 
def twistPub():
  rospy.init_node("test_publisher_twist_stamped", anonymous=True)
  
  pub = rospy.Publisher("/arm_controller/command_twist_stamped", TwistStamped, queue_size=1)
  twist_stamped_msg =  TwistStamped()

  twist_stamped_msg.header.frame_id = "base_link"
  #twist_stamped_msg.header.frame_id = "arm_7_link"
  #twist_stamped_msg.header.frame_id = "odom_combined"
  twist_stamped_msg.twist.linear.x = 0
  twist_stamped_msg.twist.linear.y = 0
  twist_stamped_msg.twist.linear.z = 0
  twist_stamped_msg.twist.angular.x = 0
  twist_stamped_msg.twist.angular.y = 0
  twist_stamped_msg.twist.angular.z = 0
   
  r = rospy.Rate(10)
  
  while not rospy.is_shutdown():
    twist_stamped_msg.header.stamp = rospy.Time.now()
    pub.publish(twist_stamped_msg)
    r.sleep()

if __name__ == '__main__':
  try:
      twistPub()
  except rospy.ROSInterruptException: pass
