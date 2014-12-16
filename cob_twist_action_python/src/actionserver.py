#!/usr/bin/python

import roslib; roslib.load_manifest('cob_twist_action_python')
import rospy
import actionlib
import sys
from geometry_msgs.msg import Twist
import cob_twist_action_python.msg
import tf
from urdf_parser_py.urdf import URDF, Robot
#from pykdl_utils.kdl_parser import *
from urdf_parser_py import urdf
from std_msgs.msg import Int32
#import orocos_kdl
import PyKDL
import urdf_parser_py
from urdf_parser_py.urdf import URDF

class TrackingActionServer(object):
    def __init__(self):
        # parse robot_description and generate KDL chains
        #if rospy.has_param('/robot_description'):
        #     self.robot_desc = rospy.get_param('/robot_description')
        #else:
        #    rospy.logerr("robot description not found")
        #    sys.exit()    
        #robot = URDF.from_parameter_server(key='robot_description')    
        #print robot
        #robot = URDF.from_parameter_server(rospy.get_param('/robot_description'))
        #robot = URDF.from_xml_string(self.robot_desc)
        
        #tree = kdl_tree_from_urdf_model(robot)
        #print self.robot_desc
       # robot = URDF.from_xml_string(self.robot_desc)
        #PyKDL.Chain.
        #_joints = PyKDL.Chain.getNrOfJoints()
        #_chain = PyKDL.Tree.getChain(string, string)    
        
        #from urdf_parser_py.urdf import URDF
        #from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
        #urdf.
        #robot = URDF.load_from_parameter_server(verbose=False)
        #tree = kdl_tree_from_urdf_model(robot)
        #print tree.getNrOfSegments()
        #chain = tree.getChain(base_link, end_link)
        #print chain.getNrOfJoints()
        #robot_desc_string = rospy.get_param("/robot_description")   
#         from urdf_parser_py.urdf import URDF
#         from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
#         robot = URDF.load_from_parameter_server(verbose=False)
#         tree = kdl_tree_from_urdf_model(robot)
        
#         if rospy.has_param('cartesian_controller/active_frame'):
#             self.active_frame = rospy.get_param("cartesian_controller/active_frame")
#         else:
#             rospy.logerr("No active_frame specified. Aborting!")
#             sys.exit()
        #tree = kdl_tree_from_urdf_model(robot)   
        #pykdl_utils.kdl_parser    
        
        ##############################
        # get x_target and x_current
        self.listener = tf.TransformListener()
        self.root_frame = "torso_center_link"
        self.active_frame = "torso_center_target"
        (trans, rot) = self.getTransformX()
        print (trans, rot)
        ###############################
        # get x_dot_target 
        rospy.Subscriber("torso/twist_controller/command_twist", Twist , callback=self.cb_twist_command, queue_size = 1)
        
        ###############################
        # create ActionServer
        self._action_name = "TwistTrackingAction"
        self._feedback = cob_twist_action_python.msg.TrackingFeedback()
        self._result = cob_twist_action_python.msg.TrackingResult()
        self._goal = cob_twist_action_python.msg.TrackingGoal()
        self._as = actionlib.SimpleActionServer(self._action_name, cob_twist_action_python.msg.TrackingAction, execute_cb=None, auto_start=False)
        self._as.register_goal_callback(self.goal_cb())
        self._as.start()
        
        
    def goal_cb(self):
        rospy.loginfo("processing new goal callback")
        if (self._as.is_new_goal_available()):
            self._as.accept_new_goal()
            rospy.loginfo("Server accepted a new goal ....")    
    
    def cb_twist_command(self, twist_data):
        print "twist wird ausgefuehrt"
        print twist_data
        
            
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
        
    def getTransformX(self):
        self.transform_available = False
        while not self.transform_available:
            try:
                (trans,rot) = self.listener.lookupTransform(self.root_frame, self.active_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #rospy.logwarn("Waiting for transform...")
                rospy.sleep(0.1)
                continue
            self.transform_available = True
            return (trans, rot)    
        
    def run(self):
        r = rospy.Rate(5)
        rospy.loginfo("ActionServerNode running ...")  
        #(trans, rot) = self.getTransformX() 
        #print (trans, rot)
        
        r.sleep()
        
        
########################################################################
#
#    Main:
#    - start node
#
if __name__ == "__main__":
    rospy.init_node('cob_twist_action_python', anonymous=False)
    tAS = TrackingActionServer() #, sys.argv[2]
    while not rospy.is_shutdown():
        tAS.run()        
            