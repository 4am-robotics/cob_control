#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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
