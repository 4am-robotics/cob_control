#! /usr/bin/env python
import math
import rospy

from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import Profile
from simple_script_server.simple_script_server import simple_script_server
import simple_cartesian_interface as sci


def init_pos():
    sss = simple_script_server()
    sss.move("arm", [[-0.0002934322105607734, -0.38304632633953606, 6.931483707006691e-07, 0.8526320037121202, 5.69952198326007e-07, -0.47039657856235184, -0.00029228225570943067]])

if __name__ == '__main__':
    rospy.init_node('test_move_circ_interface')
    init_pos()


    pose = sci.gen_pose(pos=[-0.2, 0, 0.8], rpy=[0, 0.0, 0])
    profile = Profile()
    profile.vel = 0.2
    profile.accl = 0.1
    profile.profile_type = Profile.SINOID
    #profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    #init_pos()


    pose = sci.gen_pose(pos=[0.0, 0.0, 0.8], rpy=[0.0, 0.0, 0])
    start_angle = 180.0 * math.pi / 180.0
    end_angle = 0.0 * math.pi / 180.0
    profile = Profile()
    profile.vel = 0.4
    profile.accl = 0.3
    profile.profile_type = Profile.SINOID
    #profile.profile_type = Profile.RAMP

    success, message = sci.move_circ(pose, "world", start_angle, end_angle, 0.2, profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)


    pose = sci.gen_pose(pos=[0.2, 0.0, 0.8], rpy=[0, math.pi/2, 0])
    profile = Profile()
    profile.vel = 0.3
    profile.accl = 0.3
    profile.profile_type = Profile.SINOID
    #profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    #init_pos()


    pose = sci.gen_pose(pos=[0.0, -0.2, 0.8], rpy=[0, 0.0, math.pi/2])
    profile = Profile()
    profile.vel = 0.3
    profile.accl = 0.3
    profile.profile_type = Profile.SINOID
    #profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    #init_pos()




    pose = sci.gen_pose(pos=[0.0, 0.0, 0.8], rpy=[0.0, 0.0, math.pi/2])
    start_angle = 180.0 * math.pi / 180.0
    end_angle = 0.0 * math.pi / 180.0
    profile = Profile()
    profile.vel = 0.4
    profile.accl = 0.3
    profile.profile_type = Profile.SINOID
    #profile.profile_type = Profile.RAMP

    success, message = sci.move_circ(pose, "world", start_angle, end_angle, 0.2, profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)




    pose = sci.gen_pose(pos=[0.0, 0.2, 0.8], rpy=[0, math.pi/2, 0])
    profile = Profile()
    profile.vel = 0.3
    profile.accl = 0.3
    profile.profile_type = Profile.SINOID
    #profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    #init_pos()



    pose = sci.gen_pose(pos=[-0.2, 0.0, 0.8], rpy=[0, 0, 0])
    profile = Profile()
    profile.vel = 0.3
    profile.accl = 0.3
    profile.profile_type = Profile.SINOID
    #profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)

    init_pos()


