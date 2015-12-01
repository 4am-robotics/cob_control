#!/usr/bin/env python
import rospy
import math
import random
import numpy as np
from sympy import *
from std_msgs.msg import Float64


def inputPub():
  rospy.init_node("test_publisher_moving_average", anonymous=True)

  input_pub = rospy.Publisher("input", Float64, queue_size=1)
  rospy.sleep(1.0)

  freq = 100.0
  r = rospy.Rate(freq)

  a = 0.05#2.0
  b = 0.1 * (2.0*math.pi/freq)
  c = 0.0#-math.pi/2.0
  d = 0.0#a
  x = Symbol('x')
  #a = Symbol('a')
  #b = Symbol('b')
  #c = Symbol('c')
  #d = Symbol('d')
  i = 0.0
  f_input = a*sin(b*x+c) + d

  input_msg = Float64()

  while not rospy.is_shutdown():
    input_msg.data = f_input.subs({'x': i}).evalf()
    
    noise = random.uniform(-a, a)
    #noise = random.uniform(-0.01, 0.01)
    input_msg.data += noise
    
    input_pub.publish(input_msg)
    
    i += 1.0
    r.sleep()

if __name__ == '__main__':
  try:
      inputPub()
  except rospy.ROSInterruptException: pass
