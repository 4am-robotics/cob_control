#!/usr/bin/env python

import copy
import argparse
import math
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped, Pose
from geometry_msgs.msg import Transform, TransformStamped, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
# import cob_hardware_emulation

class EmulationOdomLaser(object):
    def __init__(self, odom_frame):
        # this node emulates a very basic navigation including move_base action and localization
        #
        # interfaces
        # - subscribers:
        #   - /initialpose [geometry_msgs/PoseWithCovarianceStamped]
        # - publishers:
        #   - tf (map --> odom_frame)
        # - actions:
        #   - move_base [move_base_msgs/MoveBaseAction] (optional)


        # TODO
        # - speed factor
        # - handle cancel on move_base action

        self._odom_frame = odom_frame

        rospy.Subscriber("/base/twist_controller/command", Twist, self._twist_callback, queue_size=1)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.odometry_callback, queue_size=1)
        self._odom_publisher = rospy.Publisher("/scan_odom_node/scan_odom/scan_matcher_odom", Odometry, queue_size=1)

        self._transform_broadcaster = tf2_ros.TransformBroadcaster()

        self._timestamp_last_update = rospy.Time.now()

        self._twist = Twist()
        self._timestamp_last_twist = rospy.Time(0)

        self._odom = Odometry()
        self._odom.header.frame_id = self._odom_frame
        self._odom.child_frame_id = "base_footprint"
        self._odom.pose.pose.orientation.w = 1 # initialize orientation with a valid quaternion

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("Emulation for laser odometry running")

    def odometry_callback(self, msg):
        odometry = msg
        odometry.header.frame_id = self._odom_frame
        self._odom_publisher.publish(odometry)

    def twist_callback(self, msg):
        self._twist = msg
        self._timestamp_last_twist = rospy.Time.now()

    def timer_cb(self, event):
        # move robot (calculate new pose)
        dt = rospy.Time.now() - self._timestamp_last_update
        self._timestamp_last_update = rospy.Time.now()
        time_since_last_twist = rospy.Time.now() - self._timestamp_last_twist

        #print "dt =", dt.to_sec(), ". duration since last twist =", time_since_last_twist.to_sec()
        # we assume we're not moving any more if there is no new twist after 0.1 sec
        if time_since_last_twist < rospy.Duration(0.1):
            new_pose = copy.deepcopy(self._odom.pose.pose)
            yaw = tf.transformations.euler_from_quaternion([self._odom.pose.pose.orientation.x, self._odom.pose.pose.orientation.y, self._odom.pose.pose.orientation.z, self._odom.pose.pose.orientation.w])[2] + self._twist.angular.z * dt.to_sec()
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            new_pose.orientation.x = quat[0]
            new_pose.orientation.y = quat[1]
            new_pose.orientation.z = quat[2]
            new_pose.orientation.w = quat[3]
            new_pose.position.x += self._twist.linear.x * dt.to_sec() * math.cos(yaw) - self._twist.linear.y * dt.to_sec() * math.sin(yaw)
            new_pose.position.y += self._twist.linear.x * dt.to_sec() * math.sin(yaw) + self._twist.linear.y * dt.to_sec() * math.cos(yaw)
            self._odom.pose.pose = new_pose

            # we're moving, so we set a non-zero twist
            self._odom.twist.twist.linear.x = self._twist.linear.x
            self._odom.twist.twist.linear.y = self._twist.linear.y
            self._odom.twist.twist.angular.z = self._twist.angular.z
        else:
            # reset twist as we're not moving anymore
            self._odom.twist.twist = Twist()

        # publish tf
        # pub base_footprint --> odom_frame
        t_odom = TransformStamped()
        t_odom.header.stamp = rospy.Time.now()
        t_odom.header.frame_id = self._odom_frame
        t_odom.child_frame_id = "base_footprint"
        t_odom.transform.translation = self._odom.pose.pose.position
        t_odom.transform.rotation = self._odom.pose.pose.orientation

        transforms = [t_odom]

        self._transform_broadcaster.sendTransform(transforms)

if __name__ == '__main__':
    rospy.init_node('emulation_odom_laser')
    parser = argparse.ArgumentParser(conflict_handler='resolve',
                                     description="Tool for emulating nav by providing localization and move base interface")
    parser.add_argument('-o', '--odom_frame', help='odom frame name (default: \'odom_combined\')', default='odom_combined')
    args, unknown = parser.parse_known_args()
    rospy.loginfo("emulation_odom_laser running!")
    EmulationOdomLaser(args.odom_frame)
    rospy.spin()
