#!/usr/bin/env python

import copy
import math

import rospy
import tf
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult
from nav_msgs.msg import Odometry

class EmulationMoveBase():
    def __init__(self):
        # TODO
        # - topic interface move_base_simple/goal
        # - service reset
        # - speed factor
        # - action preemption and cancel
        # - add 2D Pose Estimation through topic


        action_name = "/move_base"

        self.as_move_base = actionlib.SimpleActionServer(action_name, MoveBaseAction, execute_cb=self.move_base_cb, auto_start = False)
        self.pub_odom = rospy.Publisher("/base/odometry_controller/odometry", Odometry, queue_size=1)
        self.br = tf.TransformBroadcaster()

        self.odom = Odometry()
        self.odom.header.frame_id = "odom_combined"
        self.odom.child_frame_id = "base_footprint"
        self.odom.pose.pose.orientation.w = 1 # initialize orientation with a valid quaternion

        self.as_move_base.start()

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("Emulation running for action %s of type MoveBaseAction"%(action_name))

    def move_base_cb(self, goal):
        rospy.loginfo("got new move_base goal: %f %f %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, tf.transformations.euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])[2])

        # get current pose
        current_pose = copy.deepcopy(self.odom.pose.pose)

        # calculate distance from current_pose to goal
        d_x = goal.target_pose.pose.position.x - current_pose.position.x
        d_y = goal.target_pose.pose.position.y - current_pose.position.y
        d_yaw = tf.transformations.euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])[2] - tf.transformations.euler_from_quaternion([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])[2]

        # calculate new pose incrementally
        default_vel = 1.0 # m/s or rad/s
        movement_time = max(math.sqrt(d_x**2 + d_y**2)/default_vel, math.fabs(d_yaw/default_vel))
        freq = 10.0
        increments = round(movement_time*freq)
        r = rospy.Rate(freq)
        new_pose = current_pose
        increment = 1
        if increments > 0:
            while not rospy.is_shutdown():
                new_pose.position.x = self.odom.pose.pose.position.x + d_x / increments
                new_pose.position.y = self.odom.pose.pose.position.y + d_y / increments
                yaw = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])[2] + d_yaw / increments
                quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
                new_pose.orientation.x = quat[0]
                new_pose.orientation.y = quat[1]
                new_pose.orientation.z = quat[2]
                new_pose.orientation.w = quat[3]
                rospy.logdebug("increment %d of %d with new pose: %f %f %f"%(increment, increments, new_pose.position.x, new_pose.position.y, yaw))
                self.odom.pose.pose = new_pose
                # we're moving, so we set a non-zero twist
                self.odom.twist.twist.linear.x = d_x / movement_time
                self.odom.twist.twist.linear.y = d_y / movement_time
                self.odom.twist.twist.angular.z = d_yaw / movement_time

                if increment >= increments:
                    break
                increment += 1
                r.sleep()
        else:
            self.odom.pose.pose = new_pose

        # reset twist as we're not moving anymore
        self.odom.twist.twist = Twist()

        # check if we reached the target goal
        #if self.odom == goal:
        #    self.as_move_base.set_succeeded(MoveBaseActionResult())
        #else:
        #    self.as_move_base.set_aborted(MoveBaseActionResult())
        rospy.loginfo("goal reached in %f seconds", movement_time)
        self.as_move_base.set_succeeded(MoveBaseActionResult())

    def timer_cb(self, event):
        # publish odometry
        msg = copy.deepcopy(self.odom)
        msg.header.stamp = rospy.Time.now() # update to current time stamp
        self.pub_odom.publish(msg)

        # publish tf
        # pub base_footprint --> odom_combined
        self.br.sendTransform((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 0),
                     (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "base_footprint",
                     "odom_combined")

        # pub odom_combined --> map
        # we emulate 'perfect' odometry, so /odom_combined is always the same as /map
        self.br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "odom_combined",
                     "map")

if __name__ == '__main__':
    rospy.init_node('emulation_move_base')
    EmulationMoveBase()
    rospy.spin()
