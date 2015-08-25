#! /usr/bin/env python
import math
import rospy

from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import Profile
import simple_cartesian_interface as sci

if __name__ == '__main__':
    rospy.init_node('test_move_lin_interface')

    pose = sci.gen_pose(pos=[-0.9, 0.0, 0.0], rpy=[0.0, 0.0, 0.0])
    profile = Profile()
    profile.vel = 0.2
    profile.accl = 0.1
    #profile.profile_type = Profile.SINOID
    profile.profile_type = Profile.RAMP

    success, message = sci.move_lin(pose, "world", profile)
    if success:
        rospy.loginfo(message)
    else:
        rospy.logerr(message)
