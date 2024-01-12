#!/usr/bin/env python

import argparse
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class EmulationOdomLaser(object):
    def __init__(self, odom_frame):
        # this node emulates the laser odom based on the odometry from the base controller
        #
        # interfaces
        # - subscribers:
        #   - /base/odometry_controller/odometry [nav_msgs/Odometry]
        # - publishers:
        #   - /scan_odom_node/scan_odom/odometry [nav_msgs/Odometry]
        #   - tf (map --> odom_frame)


        # TODO
        # - speed factor

        self._odom_frame = odom_frame

        self._transform_broadcaster = tf2_ros.TransformBroadcaster()

        self._odom = Odometry()
        self._odom.header.frame_id = self._odom_frame
        self._odom.child_frame_id = "base_footprint"
        self._odom.pose.pose.orientation.w = 1 # initialize orientation with a valid quaternion

        self._odom_publisher = rospy.Publisher("/scan_odom_node/scan_odom/odometry", Odometry, queue_size=1)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.odometry_callback, queue_size=1)
        rospy.loginfo("Emulation for laser odometry running")

    def odometry_callback(self, msg):
        self._odom = msg
        self._odom.header.frame_id = self._odom_frame
        self._odom_publisher.publish(self._odom)
        self.publish_tf()

    def publish_tf(self):
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
