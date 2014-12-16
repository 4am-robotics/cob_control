#! /usr/bin/env python

import roslib; roslib.load_manifest('actionlib_tutorials')
import roslib; roslib.load_manifest('test_python_action')
import rospy

# read arguments
import sys
import tf

import actionlib
from geometry_msgs.msg import Twist

import std_msgs
from std_msgs.msg import Int32
from std_msgs.msg import String
#from test_python_action.msg import *
import test_python_action.msg
import actionlib_tutorials.msg
from geometry_msgs.msg import Twist
import geometry_msgs

class TrackingAction(object):

    def __init__(self):
        
        # read params:
        rospy.get_param()
        # create messages
        self._feedback = test_python_action.msg.TrackingFeedback()
        self._result = test_python_action.msg.TrackingResult()
        self._goal = test_python_action.msg.TrackingGoal()
        self._result.success = False
        self._aborted = False
        self.counter = 0
        self.twist = [0.0,0.0,0.0,0.0,0.0,0.0]
        rospy.Subscriber("saft", Int32 , callback=self.cb_sub, queue_size = 1)
        self._action_name = "test"
        self._as = actionlib.SimpleActionServer(self._action_name, test_python_action.msg.TrackingAction, execute_cb=None, auto_start=False)
        self._as.register_goal_callback(self.goal_cb())
        #self._as.register_preempt_callback(self.preemt_cb())
        self._as.start()
        
    #def execute_cb(self, goal):
    #    # helper variables
    #    rospy.loginfo("execute callback")
    #    r = rospy.Rate(1)
    #    success = True
    #    self._feedback.twist = self.twist
    #    # create feedback
        
    def goal_cb(self):
        rospy.loginfo("processing new goal callback")
        if (self._as.is_new_goal_available()):
            self._as.accept_new_goal()
            rospy.loginfo("Server accepted a new goal ....")
        # data_count_
        # check wheter to accept this goal:
        
        


        
    def preemt_cb(self):    
        rospy.loginfo("is preemted from "+self._action_name)
        self._as.set_preempted(None, "preemted blabla")
        rospy.loginfo("do nice things here -------------------------")

    def cb_sub(self, data):
        if not (self._as.is_active()):
            rospy.logwarn("subscriber callback but as is not active")
            return
        
        rospy.loginfo("subscriber callback and as is active")
        self._feedback.twist.linear.x = 10
        self._feedback.twist.linear.y = 20
        self._feedback.twist.linear.z = 30
        self._feedback.twist.angular.x = 40
        self._feedback.twist.angular.y = 50
        self._feedback.twist.angular.z = 60
        rospy.loginfo("feedback was published")
        self._as.publish_feedback(self._feedback)
        self.counter = self.counter + data
        rospy.loginfo("counter says: "+self.counter)
        if (self.counter > 420):
            self.success = True
        
        
        if (self._aborted == True):
            self._result.success = False
            self._as.set_aborted(self._result," result is ready but goal is aborted")
            
        if (self.success == True):
            self._result.success = True
            self._as.set_succeeded(self._result, "result is ready and goal is succeeded")
            
        
    def run(self):
        rospy.loginfo("ActionServerNode running ...")
        r = rospy.Rate(10)
        self.counter+=1


        #self._as.internal_goal_callback(goal)
            
            
        self.printServerState()
        #self._as.register_goal_callback(self.goal_cb())
        if (self._as.is_new_goal_available()):
            # check goal
            rospy.loginfo("saft")
#             self.goal_cb()
            
        if (self._as.is_preempt_requested()):
            rospy.loginfo("set server to preemted state")
            self._as.set_preempted(None, "set Server preemted")
            
        if (self._aborted):
            self._as.set_aborted(None, "Goal tracking was aborted!")
            self._aborted = False
                
#         if (self._as.is_preempt_requested() and self._as.is_active()):
#             print "preemt request ................"
#             self._as.current_goal.set_cancel_requested()
# #             self._as.set_preempted(None, "preemted blabla")
#             
#         if (self._as.is_preempt_requested() and not self._as.is_active()):
#             print "finally cancel it "
#             self._as.current_goal.set_canceled(None, "finally canceled")
        # whatever
        r.sleep()
        
    # only for debbuing    
    def printServerState(self):
        print "---------------- SERVER STATUS ---------------"
        print "is Active: "+str(self._as.is_active())
        print "is PreemptRequested: "+str(self._as.is_preempt_requested())
        print "new goal aviable: "+str(self._as.is_new_goal_available())
        print "CURRENT GOAL: "
        print self._as.current_goal.get_goal()
        print "NEXT GOAL: "
        print self._as.next_goal.get_goal()
        print "----------------  GOAL STATUS ----------------"
        print self._as.current_goal.get_goal_status()

########################################################################
#
#    Main:
#    - start node
#
if __name__ == "__main__":
    rospy.init_node('test_python_action', anonymous=False)
    tA = TrackingAction() #, sys.argv[2]
    while not rospy.is_shutdown():
        tA.run()        
            