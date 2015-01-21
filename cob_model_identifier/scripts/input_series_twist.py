#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import TwistStamped
finished_timer = 0
initial_time=0
 
def twistPub():
  rospy.init_node("velPub", anonymous=True)

  first_cycle=True
  
  global finished_timer
  
  pub = rospy.Publisher("/arm_left/twist_controller/command_twist_stamped", TwistStamped, queue_size=1)
  twist_stamped_msg =  TwistStamped()

  twist_stamped_msg.header.frame_id = "arm_left_base_link"
  
  twist_stamped_msg.twist.linear.x = 0
  twist_stamped_msg.twist.linear.y = 0
  twist_stamped_msg.twist.linear.z = 0
  twist_stamped_msg.twist.angular.x = 0
  twist_stamped_msg.twist.angular.y = 0
  twist_stamped_msg.twist.angular.z = 0  
   
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
      twist_stamped_msg.twist.linear.x = 0.05
      twist_stamped_msg.twist.linear.y = 0
      twist_stamped_msg.twist.linear.z = 0
      twist_stamped_msg.twist.angular.x = 0
      twist_stamped_msg.twist.angular.y = 0
      twist_stamped_msg.twist.angular.z = 0
      
    if finished_timer==2:
      twist_stamped_msg.twist.linear.x = 0
      twist_stamped_msg.twist.linear.y = 0
      twist_stamped_msg.twist.linear.z = 0
      twist_stamped_msg.twist.angular.x = 0
      twist_stamped_msg.twist.angular.y = 0
      twist_stamped_msg.twist.angular.z = 0
      
    if finished_timer==3:
      twist_stamped_msg.twist.linear.x = 0.0
      twist_stamped_msg.twist.linear.y = 0.05
      twist_stamped_msg.twist.linear.z = 0
      twist_stamped_msg.twist.angular.x = 0
      twist_stamped_msg.twist.angular.y = 0
      twist_stamped_msg.twist.angular.z = 0
      
    if finished_timer==4:
      twist_stamped_msg.twist.linear.x = 0
      twist_stamped_msg.twist.linear.y = 0
      twist_stamped_msg.twist.linear.z = 0
      twist_stamped_msg.twist.angular.x = 0
      twist_stamped_msg.twist.angular.y = 0
      twist_stamped_msg.twist.angular.z = 0
      
    if finished_timer==5:
      twist_stamped_msg.twist.linear.x = 0
      twist_stamped_msg.twist.linear.y = 0
      twist_stamped_msg.twist.linear.z = 0.05
      twist_stamped_msg.twist.angular.x = 0
      twist_stamped_msg.twist.angular.y = 0
      twist_stamped_msg.twist.angular.z = 0
      
    if finished_timer==6:
      twist_stamped_msg.twist.linear.x = 0
      twist_stamped_msg.twist.linear.y = 0
      twist_stamped_msg.twist.linear.z = 0
      twist_stamped_msg.twist.angular.x = 0
      twist_stamped_msg.twist.angular.y = 0
      twist_stamped_msg.twist.angular.z = 0
    
    twist_stamped_msg.header.stamp = rospy.Time.now()
    pub.publish(twist_stamped_msg)
    #print twist_ms

    r.sleep()

if __name__ == '__main__':
  try:
      twistPub()
  except rospy.ROSInterruptException: pass
