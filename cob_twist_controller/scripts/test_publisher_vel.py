#!/usr/bin/env python
import rospy
import math
import numpy as np
from sympy import *
from std_msgs.msg import Float64MultiArray


def velPub():
  rospy.init_node("test_publisher_vel", anonymous=True)

  pub_vel = rospy.Publisher("joint_group_velocity_controller/command", Float64MultiArray, queue_size=10)
  pub_acc = rospy.Publisher("joint_group_velocity_controller/acceleration", Float64MultiArray, queue_size=10)
  rospy.sleep(1.0)

  freq = 100.0
  r = rospy.Rate(freq)
  
  joint_idx = 2

  a = 0.1#2.0
  b = 0.1 * (2.0*math.pi/freq)
  c = 0.0 #math.pi
  d = 0.0
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
  
  vel_msg = Float64MultiArray()
  vel_msg.data = [0.0] * 7
  acc_msg = Float64MultiArray()
  acc_msg.data = [0.0] * 7

  while not rospy.is_shutdown():
    vel = f_vel.subs({'x': i}).evalf()
    acc = f_acc.subs({'x': i}).evalf()
    #print("vel: ", vel)
    #print("acc: ", acc)
    vel_msg.data[joint_idx] = vel
    acc_msg.data[joint_idx] = acc
    pub_vel.publish(vel_msg)
    pub_acc.publish(acc_msg)
    i += 1.0
    r.sleep()

if __name__ == '__main__':
  try:
      velPub()
  except rospy.ROSInterruptException: pass
