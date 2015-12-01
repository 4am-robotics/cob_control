#! /usr/bin/env python
import math
import rospy

from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import Profile
from simple_script_server.simple_script_server import simple_script_server
import simple_cartesian_interface as sci


def init_pos():
    sss = simple_script_server()
    sss.move("arm", [[-0.00032004093963244884, -0.7064191894021441, -1.577532922958369e-06, 1.4183686971944311, 1.2084352562169443e-05, -0.6913530502577565, -0.0002663056533762642]])

if __name__ == '__main__':
    rospy.init_node('test_move_lin_interface')
    init_pos()


    pose = sci.gen_pose(pos=[0.0, 0, 0.9], rpy=[0, 0.0, 0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.05
    #profile.profile_type = Profile.SINOID
    profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    #init_pos()
    rospy.sleep(3.0)


    pose = sci.gen_pose(pos=[0.3, 0, 0.9], rpy=[0, 0.0, 0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.05
    #profile.profile_type = Profile.SINOID
    profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    #init_pos()
    rospy.sleep(3.0)
    

    pose = sci.gen_pose(pos=[0.3, 0, 0.8], rpy=[0, 0.0, 0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.05
    #profile.profile_type = Profile.SINOID
    profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    #init_pos()
    rospy.sleep(3.0)



    pose = sci.gen_pose(pos=[-0.3, 0, 0.8], rpy=[0, 0.0, 0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.05
    #profile.profile_type = Profile.SINOID
    profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    #init_pos()
    rospy.sleep(3.0)


    pose = sci.gen_pose(pos=[-0.3, 0, 0.9], rpy=[0, 0.0, 0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.05
    #profile.profile_type = Profile.SINOID
    profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    #init_pos()
    rospy.sleep(2.0)


     
    pose = sci.gen_pose(pos=[0.0, 0, 0.9], rpy=[0, 0.0, 0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.05
    #profile.profile_type = Profile.SINOID
    profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    #init_pos()
    rospy.sleep(3.0)
