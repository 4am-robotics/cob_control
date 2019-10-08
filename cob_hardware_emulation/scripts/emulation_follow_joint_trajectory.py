#!/usr/bin/env python

import copy

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState

class EmulationFollowJointTrajectory():
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

        rospy.loginfo("Emulation running for action %s of type FollowJointTrajectoryAction"%(action_name))

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

                js = copy.deepcopy(self.joint_states)

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

                # this loop samples the time segment from the current states to the next goal state in "points"
                while not rospy.is_shutdown() and ((point.time_from_start - latest_time_from_start) > rospy.Duration(0)):  
                    # current time passed in local duration segment 
                    t0 = latest_time_from_start - time_since_start_of_previous_point
                    # compute the interpolation weight as a fraction of passed time and upper bound time in this local segment  
                    alpha = t0 / t1                        
                    # interpolate linearly (lerp) each component
                    interpolated_positions = copy.deepcopy(js.position)
                    for i in range(len(point.positions)):                        
                        interpolated_positions[i] = (1.0 - alpha) * js.position[i] + alpha * point.positions[i]   
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
