#!/usr/bin/env python

import argparse
import copy
import math
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

class EmulationOdomLaser(object):
    def __init__(self, odom_frame):
        # this node emulates the laser odom based on the odometry from the base controller
        #
        # interfaces
        # - subscribers:
        #   - /base/odometry_controller/odometry [nav_msgs/Odometry]
        # - publishers:
        #   - /scan_odom_node/scan_odom/scan_matcher_odom [nav_msgs/Odometry]
        #   - tf (map --> odom_frame)


        # TODO
        # - speed factor

        self._odom_frame = odom_frame

        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.odometry_callback, queue_size=1)
        self._odom_publisher = rospy.Publisher("/scan_odom_node/scan_odom/scan_matcher_odom", Odometry, queue_size=1)

        self._transform_broadcaster = tf2_ros.TransformBroadcaster()

        self._timestamp_last_update = rospy.Time.now()
        self._timestamp_last_odom = rospy.Time(0)

        self._odom = Odometry()
        self._odom.header.frame_id = self._odom_frame
        self._odom.child_frame_id = "base_footprint"
        self._odom.pose.pose.orientation.w = 1 # initialize orientation with a valid quaternion

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("Emulation for laser odometry running")

    def odometry_callback(self, msg):
        self._odom = msg
        self._odom.header.frame_id = self._odom_frame
        self._odom_publisher.publish(self._odom)

    def timer_cb(self, event):
        # move robot (calculate new pose)
        dt = rospy.Time.now() - self._timestamp_last_update
        self._timestamp_last_update = rospy.Time.now()
        time_since_last_odom = rospy.Time.now() - self._timestamp_last_odom

        #print "dt =", dt.to_sec(), ". duration since last twist =", time_since_last_odom.to_sec()
        # we assume we're not moving any more if there is no new twist after 0.1 sec
        if time_since_last_odom < rospy.Duration(0.1):
            new_pose = copy.deepcopy(self._odom.pose.pose)
            yaw = tf.transformations.euler_from_quaternion([self._odom.pose.pose.orientation.x, self._odom.pose.pose.orientation.y, self._odom.pose.pose.orientation.z, self._odom.pose.pose.orientation.w])[2] + self._odom.twist.twist.angular.z * dt.to_sec()
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            new_pose.orientation.x = quat[0]
            new_pose.orientation.y = quat[1]
            new_pose.orientation.z = quat[2]
            new_pose.orientation.w = quat[3]
            new_pose.position.x += self._odom.twist.twist.linear.x * dt.to_sec() * math.cos(yaw) - self._odom.twist.twist.linear.y * dt.to_sec() * math.sin(yaw)
            new_pose.position.y += self._odom.twist.twist.linear.x * dt.to_sec() * math.sin(yaw) + self._odom.twist.twist.linear.y * dt.to_sec() * math.cos(yaw)
            self._odom.pose.pose = new_pose

            # we're moving, so we set a non-zero twist
            self._odom.twist.twist.linear.x = self._odom.twist.twist.linear.x
            self._odom.twist.twist.linear.y = self._odom.twist.twist.linear.y
            self._odom.twist.twist.angular.z = self._odom.twist.twist.angular.z
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
                                     description="Tool for emulating laser odom based on the odometry from the base controller")
    parser.add_argument('-o', '--odom_frame', help='odom frame name (default: \'odom_combined\')', default='odom_combined')
    args, unknown = parser.parse_known_args()
    rospy.loginfo("emulation_odom_laser running!")
    EmulationOdomLaser(args.odom_frame)
    rospy.spin()
