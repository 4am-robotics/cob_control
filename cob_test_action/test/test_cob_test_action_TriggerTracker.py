#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_test_action')
import rospy

import actionlib
import yaml
from cob_test_action.msg import *

test_TriggerTracker_goal = open("test/testdata_cob_test_action_TriggerTracker", 'r')

def TriggerTracker_client():
    client = actionlib.SimpleActionClient('/TriggerTracker', TriggerAction)
    client.wait_for_server()
    
    goal =TriggerGoal()
    genpy.message.fill_message_args(goal, yaml.load(test_TriggerTracker_goal))

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


if __name__ == "__main__":
    rospy.init_node('TriggerTracker_test')
    print TriggerTracker_client()
