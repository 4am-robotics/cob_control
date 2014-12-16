#! /usr/bin/env python

import roslib; roslib.load_manifest('test_python_action')
import rospy

# read arguments
import sys

import actionlib
from geometry_msgs.msg import Twist

import std_msgs
from std_msgs.msg import Int32
from std_msgs.msg import String
#from test_python_action.msg import *
import test_python_action.msg 

from geometry_msgs.msg import Twist
import geometry_msgs

class TrackingActionClient(object):
    def __init__(self):
    # create messages
        self.counter = 50
        self.counter2 = 0
        self._ac = actionlib.SimpleActionClient("test", test_python_action.msg.TrackingAction)
        self._goal = test_python_action.msg.TrackingGoal()
        self._goal.tracking = True
        
        rospy.loginfo("Waiting for server to start")
        self._ac.wait_for_server()
        
        rospy.loginfo("action server started, sending first goal now ...")
        

    
    def run(self):
        rospy.loginfo("ActionServerNode running ...")
        r = rospy.Rate(10)
        
        finished_before_timeout = False
        
        self.counter+=1
        self.counter2+=1
        
        if (self.counter > 100):
            rospy.loginfo("send new goal")
            self._ac.send_goal(self._goal, done_cb=None, active_cb=None, feedback_cb=None)
            self.counter = 0
        
        if (self.counter2 > 1000):
            rospy.loginfo("cancel all goals")
            self._ac.cancel_all_goals()
            self.counter2 = 0
            
        #finished_before_timeout = self._ac.wait_for_result(rospy.Duration(30.0))

        if (finished_before_timeout):
            state = self._ac.get_state()
            rospy.loginfo("Action finished with state "+str(state))
        #else:
        #    rospy.loginfo("action did not finish in time")   
        #    rospy.signal_shutdown("Error 002 - File not found!")   
        
        # whatever
        r.sleep()     
        
        
        
               
########################################################################
#
#    Main:
#    - start node
#
if __name__ == "__main__":
    rospy.init_node('test_python_action_client', anonymous=False)
    tAC = TrackingActionClient() #, sys.argv[2]
    while not rospy.is_shutdown():
        tAC.run()   