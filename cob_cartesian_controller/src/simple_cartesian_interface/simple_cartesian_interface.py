#!/usr/bin/python
import rospy
import threading
import actionlib

from tf.transformations import *
from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile


_transform_listener = None
_transform_listener_ock = threading.Lock()

_action_client = None
_action_client_lock = threading.Lock()

def move_lin(pose_goal, frame_id, profile, rotate_only=False):
    client = actionlib.SimpleActionClient('cartesian_trajectory_action', CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer...")
    success = client.wait_for_server(rospy.Duration(2.0))
    if(not success):
        return (success, "ActionServer not available within timeout")
    
    goal = CartesianControllerGoal()
    goal.name = "move_lin"
    goal.move_lin.pose_goal = pose_goal
    goal.move_lin.frame_id = frame_id
    goal.move_lin.rotate_only = rotate_only
    goal.move_lin.profile = profile
    print goal
    
    client.send_goal(goal)
    print "goal sent"
    state = client.get_state()
    print state
    client.wait_for_result()
    print "result received"
    result = client.get_result()
    return (result.success, result.message)

def move_circ(pose_center, frame_id, start_angle, end_angle, radius, profile, rotate_only=False):
    client = actionlib.SimpleActionClient('cartesian_trajectory_action', CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer...")
    success = client.wait_for_server(rospy.Duration(2.0))
    if(not success):
        return (success, "ActionServer not available within timeout")
    
    goal = CartesianControllerGoal()
    goal.name = "move_circ"
    goal.move_circ.pose_center = pose_center
    goal.move_circ.frame_id = frame_id
    goal.move_circ.rotate_only = rotate_only
    goal.move_circ.start_angle = start_angle
    goal.move_circ.end_angle = end_angle
    goal.move_circ.radius = radius
    goal.move_circ.profile = profile
    print goal
    
    client.send_goal(goal)
    print "goal sent"
    state = client.get_state()
    print state
    client.wait_for_result()
    print "result received"
    result = client.get_result()
    return (result.success, result.message)



####################################
#  helpers
####################################

'''
Get the transform listener for this process.
'''
def get_transform_listener():
    global _transform_listener
    with _transform_listener_lock:
        if _transform_listener == None:
            _transform_listener = tf.TransformListener()
        return _transform_listener

'''
Get the action client for this process.
'''
def get_action_client():
    global _action_client
    with _action_client_lock:
        if _action_client == None:
            _action_client = actionlib.SimpleActionClient('cartesian_trajectory_action', CartesianControllerAction)
        return _action_client

'''
Generates a geometry_msgs.Pose from (x,y,z) and (r,p,y)
'''
def gen_pose(pos=[0,0,0], rpy=[0,0,0]):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = pos
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_euler(*rpy)
    return pose
