#!/usr/bin/env python
import rospy
import math
import random
import numpy as np
from sympy import *
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
