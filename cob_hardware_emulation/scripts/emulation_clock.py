#!/usr/bin/env python

import time
import rospy
from rosgraph_msgs.msg import Clock

rospy.init_node('emulation_clock', disable_rostime=True)
pub = rospy.Publisher('/clock', Clock, queue_size=1)
time_factor = rospy.get_param('/emulation_time_factor', 1.0)
if not time_factor > 0.0:
    rospy.logerr("emulation_time_factor must be >0.0, but is {}. exiting...".format(time_factor))
    exit(-1)

t = time.time()
dt = 0.01
while not rospy.is_shutdown():
    t+=dt*time_factor
    msg = Clock()
    msg.clock = rospy.Time(t)
    pub.publish(msg)
    time.sleep(dt)