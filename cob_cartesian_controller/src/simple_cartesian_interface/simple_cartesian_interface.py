#!/usr/bin/python
import rospy
import threading
import actionlib

from tf.transformations import *
from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import CartesianControllerAction, CartesianControllerGoal
from cob_cartesian_controller.msg import Profile


def move_lin(pose_goal, frame_id, profile, rotate_only=False):
    client = actionlib.SimpleActionClient('cartesian_trajectory_action', CartesianControllerAction)
    rospy.logwarn("Waiting for ActionServer...")
    success = client.wait_for_server(rospy.Duration(2.0))
    if(not success):
        return (success, "ActionServer not available within timeout")

    goal = CartesianControllerGoal()
    goal.move_type = CartesianControllerGoal.LIN
    goal.move_lin.pose_goal = pose_goal
    goal.move_lin.frame_id = frame_id
    goal.profile = profile
    # print goal

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
    goal.move_type = CartesianControllerGoal.CIRC
    goal.move_circ.pose_center = pose_center
    goal.move_circ.frame_id = frame_id
    goal.move_circ.start_angle = start_angle
    goal.move_circ.end_angle = end_angle
    goal.move_circ.radius = radius
    goal.profile = profile
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
Generates a geometry_msgs.Pose from (x,y,z) and (r,p,y)
'''
def gen_pose(pos=[0,0,0], rpy=[0,0,0]):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = pos
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_euler(*rpy)


    return pose
