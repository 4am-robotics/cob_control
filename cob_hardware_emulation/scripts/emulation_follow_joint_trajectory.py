#!/usr/bin/env python

import actionlib
import copy
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState

class emulation():
    def __init__(self):
        # TODO
        # - service reset
        # - speed factor
        # - interpolated movement for joint states
        # - action preemption and cancel
        
        
        params = rospy.get_param('~')
        self.joint_names = params['joint_names']

        action_name = "joint_trajectory_controller/follow_joint_trajectory"

        self.as_fjta = actionlib.SimpleActionServer(action_name, FollowJointTrajectoryAction, execute_cb=self.fjta_cb, auto_start = False)
        self.pub_joint_states = rospy.Publisher("joint_states", JointState, queue_size=1)
        js = JointState()
        js.name = copy.deepcopy(self.joint_names)
        js.position = [0]*len(js.name)
        js.velocity = [0]*len(js.name)
        js.effort = [0]*len(js.name)
        self.joint_states = js

        self.as_fjta.start()

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("Emulator running for action %s of type follow_joint_trajectory"%(action_name))

    def fjta_cb(self, goal):
        joint_names = copy.deepcopy(self.joint_names)
        joint_names.sort()
        fjta_joint_names = copy.deepcopy(goal.trajectory.joint_names)
        fjta_joint_names.sort()
        if joint_names == fjta_joint_names:
            rospy.loginfo("got a new joint trajectory goal for %s", joint_names)
            # sort goal to fit joint_names order in joint_states

            goal_sorted = copy.deepcopy(goal)
            goal_sorted.trajectory.joint_names = self.joint_names

            latest_time_from_start = rospy.Duration(0)
            for point in goal_sorted.trajectory.points:

                # we need to resort the positions array because moveit sorts alphabetically but all other ROS components sort in the URDF order
                positions_sorted = []
                for joint_name in self.joint_names:
                    idx = goal.trajectory.joint_names.index(joint_name)
                    positions_sorted.append(point.positions[idx])
                point.positions = positions_sorted

                js = copy.deepcopy(self.joint_states)
                js.position = point.positions
                
                # this assumes that the joint_state flips to the new position once the time_from_start has passed. 
                # FIXME calculate interpolated ramp based on time_from_start
                rospy.sleep((point.time_from_start - latest_time_from_start).to_sec())

                latest_time_from_start = point.time_from_start
                self.joint_states = js

            self.as_fjta.set_succeeded(FollowJointTrajectoryResult())
        else:
            rospy.logerr("received unexpected joint names in goal")
            self.as_fjta.set_aborted()

    def timer_cb(self, event):
        msg = copy.deepcopy(self.joint_states)
        msg.header.stamp = rospy.Time.now() # update to current time stamp
        self.pub_joint_states.publish(msg)

if __name__ == '__main__':
    rospy.init_node('emulation')
    emulation()
    rospy.loginfo("follow joint trajectory emulation is running")
    rospy.spin()
