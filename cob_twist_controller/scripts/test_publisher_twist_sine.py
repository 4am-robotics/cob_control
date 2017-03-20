#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist

def velPub():
  rospy.init_node("test_publisher_vel", anonymous=True)

  pub_vel = rospy.Publisher("twist_controller/command_twist", Twist, queue_size=1)
  rospy.sleep(1.0)

  freq = 100.0
  r = rospy.Rate(freq)
  
  joint_idx = 1

  a = 0.05#2.0
  b = 0.1 * (2.0*math.pi/freq)
  c = -math.pi/2.0
  d = a
  i = 0.0
  
  vel_msg = Twist()

  while not rospy.is_shutdown():
    vel = a*math.sin(b*i+c) + d
    #print("vel: ", vel)
    vel_msg.angular.z = vel
    pub_vel.publish(vel_msg)
    i += 1.0
    r.sleep()

if __name__ == '__main__':
  try:
      velPub()
  except rospy.ROSInterruptException: pass
