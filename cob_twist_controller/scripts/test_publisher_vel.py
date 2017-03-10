#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64MultiArray

def velPub():
  rospy.init_node("test_publisher_vel", anonymous=True)

  pub_vel = rospy.Publisher("joint_group_velocity_controller/command", Float64MultiArray, queue_size=1)
  rospy.sleep(1.0)

  freq = 100.0
  r = rospy.Rate(freq)
  
  joint_idx = 2

  a = 0.1#2.0
  b = 0.1 * (2.0*math.pi/freq)
  c = 0.0 #math.pi
  d = 0.0
  i = 0.0
  
  vel_msg = Float64MultiArray()
  vel_msg.data = [0.0] * 7

  while not rospy.is_shutdown():
    vel = a*math.sin(b*i+c) + d
    #print("vel: ", vel)
    vel_msg.data[joint_idx] = vel
    pub_vel.publish(vel_msg)
    i += 1.0
    r.sleep()

if __name__ == '__main__':
  try:
      velPub()
  except rospy.ROSInterruptException: pass
