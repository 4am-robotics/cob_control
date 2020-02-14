#!/usr/bin/env python

import time
import rospy
from rosgraph_msgs.msg import Clock

class EmulationClock(object):
    def __init__(self):
        self.t = time.time()
        self.dt = 0.01
        self.time_factor = rospy.get_param('/emulation_time_factor', 1.0)
        if not self.time_factor > 0.0:
            rospy.logerr("emulation_time_factor must be >0.0, but is {}. exiting...".format(self.time_factor))
            exit(-1)
        self.pub = rospy.Publisher('/clock', Clock, queue_size=1)
        rospy.Timer(rospy.Duration(self.dt), self.timer_cb)

    def timer_cb(self, event):
        self.t+=self.dt*self.time_factor
        msg = Clock()
        msg.clock = rospy.Time(self.t)
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('emulation_clock', disable_rostime=True)
    EmulationClock()
    rospy.spin()
