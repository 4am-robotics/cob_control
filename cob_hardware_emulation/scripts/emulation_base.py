#!/usr/bin/env python

import copy
import math

import rospy
import tf
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult
from nav_msgs.msg import Odometry

class EmulationBase():
    def __init__(self):
        # this node emulates a base including localization
        #
        # interfaces
        # - subscribers:
        #   - /base/twist_controller/command [geometry_msgs/Twist]
        # - publishers:
        #   - /base/odometry_controller/odometry [nav_msgs/Odometry]
        #   - tf (odom_combined --> base_footprint, map --> odom_combined)


        # TODO
        # - add service reset odometry
        # - speed factor
        # - add 2D Pose Estimation through topic

        rospy.Subscriber("/base/twist_controller/command", Twist, self.twist_callback, queue_size=1)
        self.pub_odom = rospy.Publisher("/base/odometry_controller/odometry", Odometry, queue_size=1)
        self.br = tf.TransformBroadcaster()

        self.timestamp_last_update = rospy.Time.now()

        self.twist = Twist()
        self.timestamp_last_twist = rospy.Time(0)

        self.odom = Odometry()
        self.odom.header.frame_id = "odom_combined"
        self.odom.child_frame_id = "base_footprint"
        self.odom.pose.pose.orientation.w = 1 # initialize orientation with a valid quaternion

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("Emulation for base running")

    def twist_callback(self, msg):
        self.twist = msg
        self.timestamp_last_twist = rospy.Time.now()

    def timer_cb(self, event):
        # move robot (calculate new pose)
        dt = rospy.Time.now() - self.timestamp_last_update
        self.timestamp_last_update = rospy.Time.now()
        time_since_last_twist = rospy.Time.now() - self.timestamp_last_twist

        #print "dt =", dt.to_sec(), ". duration since last twist =", time_since_last_twist.to_sec()
        # we assume we're not moving any more if there is no new twist after 0.1 sec
        if time_since_last_twist < rospy.Duration(0.1):
            new_pose = copy.deepcopy(self.odom.pose.pose)
            yaw = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])[2] + self.twist.angular.z * dt.to_sec()
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            new_pose.orientation.x = quat[0]
            new_pose.orientation.y = quat[1]
            new_pose.orientation.z = quat[2]
            new_pose.orientation.w = quat[3]
            new_pose.position.x += self.twist.linear.x * dt.to_sec() * math.cos(yaw) - self.twist.linear.y * dt.to_sec() * math.sin(yaw)
            new_pose.position.y += self.twist.linear.x * dt.to_sec() * math.sin(yaw) + self.twist.linear.y * dt.to_sec() * math.cos(yaw)
            self.odom.pose.pose = new_pose

            # we're moving, so we set a non-zero twist
            self.odom.twist.twist.linear.x = self.twist.linear.x
            self.odom.twist.twist.linear.y = self.twist.linear.y
            self.odom.twist.twist.angular.z = self.twist.angular.z
        else:
            # reset twist as we're not moving anymore
            self.odom.twist.twist = Twist()

        # publish odometry
        odom = copy.deepcopy(self.odom)
        odom.header.stamp = rospy.Time.now() # update to current time stamp
        self.pub_odom.publish(odom)

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
    rospy.init_node('emulation_base')
    EmulationBase()
    rospy.spin()