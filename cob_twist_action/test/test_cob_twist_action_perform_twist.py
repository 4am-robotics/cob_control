#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_twist_action')
import rospy

import actionlib
import yaml
from cob_twist_action.msg import *

test_perform_twist_goal = open("test/testdata_cob_twist_action_perform_twist", 'r')

def perform_twist_client():
    client = actionlib.SimpleActionClient('/perform_twist', TwistAction)
    client.wait_for_server()
    
    goal =TwistGoal()
    genpy.message.fill_message_args(goal, yaml.load(test_perform_twist_goal))

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


if __name__ == "__main__":
    rospy.init_node('perform_twist_test')
    print perform_twist_client()
