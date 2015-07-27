#! /usr/bin/env python
import math
import rospy

from geometry_msgs.msg import Pose
from cob_cartesian_controller.msg import Profile
import simple_cartesian_interface as sci

if __name__ == '__main__':
    rospy.init_node('test_move_circ_interface')
    
    pose = sci.gen_pose(pos=[0.0, 0.7, 1.0], rpy=[0.0, 0.0, 0.0])
    start_angle = 0.0 * math.pi / 180.0
    end_angle = 90.0 * math.pi / 180.0
    profile = Profile()
    profile.vel = 0.2
    profile.accl = 0.1
    #profile.profile_type = Profile.SINOID
    profile.profile_type = Profile.RAMP
    
    success = sci.move_circ(pose, "world", start_angle, end_angle, 0.3, profile)
