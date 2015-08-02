#! /usr/bin/env python
from math import radians
import rospy

from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import Profile
import simple_cartesian_interface as sci

if __name__ == '__main__':
    rospy.init_node('test_move_santa')

    # Step 1
    pose = sci.gen_pose(pos=[-1.0, 0.0, 0.0], rpy=[radians(45.0), 0.0, radians(-25.0)])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.2
    profile.profile_type = Profile.SINOID
    success, message = sci.move_lin(pose, "world", profile)

    rospy.sleep(4.0)

    # Step 2
    pose = sci.gen_pose(pos=[-0.5, 0.5, 0.0], rpy=[radians(-45.0), 0.0, radians(-30.0)])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.2
    profile.profile_type = Profile.SINOID
    success, message = sci.move_lin(pose, "world", profile)

    rospy.sleep(4.0)

    # Step 3
    pose = sci.gen_pose(pos=[0.5, 0.5, 0.0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.2
    profile.profile_type = Profile.SINOID
    success, message = sci.move_lin(pose, "world", profile)

    rospy.sleep(2.0)

    # Step 4
    pose = sci.gen_pose(pos=[1.0, 0.0, 0.0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.2
    profile.profile_type = Profile.SINOID
    success, message = sci.move_lin(pose, "world", profile)

    rospy.sleep(2.0)

    # Step 5
    pose = sci.gen_pose(pos=[0.0, -1.0, 0.0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.2
    profile.profile_type = Profile.SINOID
    success, message = sci.move_lin(pose, "world", profile)

    rospy.sleep(2.0)

    # Step 6
    pose = sci.gen_pose(pos=[-1.0, 1.0, 0.0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.2
    profile.profile_type = Profile.SINOID
    success, message = sci.move_lin(pose, "world", profile)

    rospy.sleep(2.0)

    # Step 7
    pose = sci.gen_pose(pos=[0.0, -1.0, 0.0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.2
    profile.profile_type = Profile.SINOID
    success, message = sci.move_lin(pose, "world", profile)

    rospy.sleep(2.0)

    # Step 8
    pose = sci.gen_pose(pos=[1.0, 1.0, 0.0])
    profile = Profile()
    profile.vel = 0.1
    profile.accl = 0.2
    profile.profile_type = Profile.SINOID
    success, message = sci.move_lin(pose, "world", profile)
