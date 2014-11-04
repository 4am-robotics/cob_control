#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist

finished_timer = 0
initial_time=0
 
def twistPub():
  rospy.init_node("velPub", anonymous=True)
  
  print("Ejecutando")
  
  first_cycle=True
  
  global finished_timer
  
  pub = rospy.Publisher("/arm_controller/command_twist", Twist, queue_size=1)
  twist_msg =  Twist()

  twist_msg.linear.x = 0
  twist_msg.linear.y = 0
  twist_msg.linear.z = 0
  twist_msg.angular.x = 0
  twist_msg.angular.y = 0
  twist_msg.angular.z = 0  
   
  r = rospy.Rate(100) 
  last_time=0
  
  while not rospy.is_shutdown():     
    #print vel	
		
    if first_cycle:
      initial_time=rospy.get_time()
      last_time=rospy.get_time()
      first_cycle=False

    if(rospy.get_time()-last_time>6):
      last_time=rospy.get_time()
      finished_timer+=1
      print 'Finished timer:' + str(finished_timer) + 'Current_time' + str(rospy.get_time()-initial_time)

    if finished_timer==1:
      #print "First segment, time: %.3f" % (rospy.get_time()-initial_time)
      #twist_msg.linear.x = -0.04*math.sqrt(2)*math.cos(math.pi/4)
      #twist_msg.linear.y = 0.0
      #twist_msg.linear.z = -0.04*math.sqrt(2)*math.sin(math.pi/4)
      twist_msg.linear.x = -0.04
      twist_msg.linear.y = 0.0
      twist_msg.linear.z = -0.04
      twist_msg.angular.x = 0
      twist_msg.angular.y = 0
      twist_msg.angular.z = 0      
      
    if finished_timer==2:
      #print "Second segment, time: %.3f" % (rospy.get_time()-initial_time)
      #twist_msg.linear.x = 0.04
      #twist_msg.linear.z = 0
      twist_msg.linear.x = 0
      twist_msg.linear.z = 0
      twist_msg.angular.x = 0.3
    
    if finished_timer==3:
      #print "Third segment, time: %.3f" % (rospy.get_time()-initial_time)
      #twist_msg.linear.x = 0
      #twist_msg.linear.z = 0.04
      twist_msg.angular.x = 0
      twist_msg.angular.z = 0.1
	  
    if finished_timer==4:
      #print("Done")
      twist_msg.linear.x = 0
      twist_msg.linear.z = 0
      twist_msg.angular.z = 0.0
	  
      
    pub.publish(twist_msg)
    #print twist_ms

    r.sleep()

if __name__ == '__main__':
  try:
      twistPub()
  except rospy.ROSInterruptException: pass
