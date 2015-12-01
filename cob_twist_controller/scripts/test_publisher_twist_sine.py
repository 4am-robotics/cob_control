#!/usr/bin/env python
import rospy
import math
import numpy as np
from sympy import *
from geometry_msgs.msg import Twist


def velPub():
  rospy.init_node("test_publisher_vel", anonymous=True)

  pub_vel = rospy.Publisher("twist_controller/command_twist", Twist, queue_size=1)
  pub_acc = rospy.Publisher("twist_controller/command_twist_acc", Twist, queue_size=1)
  rospy.sleep(1.0)

  freq = 100.0
  r = rospy.Rate(freq)
  
  joint_idx = 1

  a = 0.05#2.0
  b = 0.1 * (2.0*math.pi/freq)
  c = -math.pi/2.0
  d = a
  x = Symbol('x')
  #a = Symbol('a')
  #b = Symbol('b')
  #c = Symbol('c')
  #d = Symbol('d')
  i = 0.0
  f_vel = a*sin(b*x+c) + d
  f_acc = f_vel.diff(x)
  print f_vel
  print f_acc
  
  vel_msg = Twist()
  acc_msg = Twist()

  while not rospy.is_shutdown():
    vel = f_vel.subs({'x': i}).evalf()
    acc = f_acc.subs({'x': i}).evalf()
    #print("vel: ", vel)
    #print("acc: ", acc)
    vel_msg.angular.z = vel
    acc_msg.angular.z = acc
    pub_vel.publish(vel_msg)
    pub_acc.publish(acc_msg)
    i += 1.0
    r.sleep()

if __name__ == '__main__':
  try:
      velPub()
  except rospy.ROSInterruptException: pass
