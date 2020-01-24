#!/usr/bin/env python

import copy

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

class EmulationFollowJointTrajectory(object):
    def __init__(self):
        # TODO
        # - speed factor

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

        # reset service
        self.service_reset_fjta = rospy.Service("reset_joint_states", Trigger, self.reset_cb)

        rospy.loginfo("Emulation running for action %s of type FollowJointTrajectoryAction"%(action_name))

    def reset_cb(self, req):
        self.joint_states.position = [0.0] * len(self.joint_states.position)
        self.joint_states.velocity = [0.0] * len(self.joint_states.velocity)
        self.joint_states.effort   = [0.0] * len(self.joint_states.effort)

        return TriggerResponse(
            success = True,
            message = "Succesfully reset joint states"
        )           
        
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

            # keep track of the current time
            latest_time_from_start = rospy.Duration(0)
            # the point in time of a previous computation.
            # This is used to compute the interpolation weight as a function of current and goal time point
            time_since_start_of_previous_point = rospy.Duration(0)

            # for all points in the desired trajectory
            for point in goal_sorted.trajectory.points:

                # we need to resort the positions array because moveit sorts alphabetically but all other ROS components sort in the URDF order
                positions_sorted = []
                for joint_name in self.joint_names:
                    idx = goal.trajectory.joint_names.index(joint_name)
                    positions_sorted.append(point.positions[idx])
                point.positions = positions_sorted
                pos_length = len(point.positions)

                joint_states_prev = copy.deepcopy(self.joint_states)

                # linear interpolation of the given trajectory samples is used
                # to compute smooth intermediate joints positions at a fixed resolution

                # fixed frequency to control the granularity of the sampling resolution
                sample_rate_hz = 10
                # duration from one sample to the next
                sample_rate_dur_secs = (1.0 / float(sample_rate_hz))
                # rospy loop rate
                sample_rate = rospy.Rate(sample_rate_hz) # 10Hz for now
                # upper bound of local duration segment
                t1 = point.time_from_start - time_since_start_of_previous_point
                # compute velocity as the fraction of distance from prev point to next point in trajectory
                # and the corresponding time t1
                velocities = [0] * pos_length
                for i in range(pos_length):
                    if t1.to_sec() != 0.0:
                        velocities[i] = (point.positions[i] - joint_states_prev.position[i]) / float(t1.to_sec())
                    else:
                        velocities[i] = 0.0
                self.joint_states.velocity = velocities

                # this loop samples the time segment from the current states to the next goal state in "points"
                while not rospy.is_shutdown() and ((point.time_from_start - latest_time_from_start) > rospy.Duration(0)):
                    # check that preempt has not been requested by the client
                    if self.as_fjta.is_preempt_requested():
                        rospy.loginfo("preempt requested")
                        self.as_fjta.set_preempted()
                        return

                    # current time passed in local duration segment
                    t0 = latest_time_from_start - time_since_start_of_previous_point
                    # compute the interpolation weight as a fraction of passed time and upper bound time in this local segment
                    if t1 != 0.0:
                        alpha = t0 / t1
                    else:
                        alpha = 0.0

                    # interpolate linearly (lerp) each component
                    interpolated_positions = [0] * pos_length
                    for i in range(pos_length):
                        interpolated_positions[i] = (1.0 - alpha) * joint_states_prev.position[i] + alpha * point.positions[i]
                    self.joint_states.position = interpolated_positions

                    # sleep until next sample update
                    sample_rate.sleep()
                    # increment passed time
                    latest_time_from_start += rospy.Duration(sample_rate_dur_secs)

                # ensure that the goal and time point is always exactly reached
                latest_time_from_start = point.time_from_start
                self.joint_states.position = point.positions
                # set lower time bound for the next point
                time_since_start_of_previous_point = latest_time_from_start
            
            # set joint velocities to zero after the robot stopped moving (reaching final point of trajectory)
            self.joint_states.velocity = [0.0] * len(self.joint_states.velocity)    
            self.joint_states.effort   = [0.0] * len(self.joint_states.effort)
            
            self.as_fjta.set_succeeded(FollowJointTrajectoryResult())
        else:
            rospy.logerr("received unexpected joint names in goal")
            self.as_fjta.set_aborted()

    def publish_joint_states(self):
        msg = copy.deepcopy(self.joint_states)
        msg.header.stamp = rospy.Time.now() # update to current time stamp
        self.pub_joint_states.publish(msg)

if __name__ == '__main__':
    rospy.init_node('emulation_follow_joint_trajectory')

    emulation_follow_joint_trajectory = EmulationFollowJointTrajectory()

    # updating the joint states: 10Hz
    joint_states_pub_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        emulation_follow_joint_trajectory.publish_joint_states()
        joint_states_pub_rate.sleep()
