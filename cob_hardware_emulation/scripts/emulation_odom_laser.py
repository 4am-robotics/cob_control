#!/usr/bin/env python

import actionlib
import argparse
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry

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

        self.odom_frame_ = odom_frame

        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initalpose_callback, queue_size=1)
        rospy.Subscriber("/base/odometry_controller/odometry", Odometry, self.odometry_callback, queue_size=1)
        self.odom_publisher_ = rospy.Publisher("/scan_odom_node/scan_odom/scan_matcher_odom", Odometry, queue_size=1)

        self.br = tf2_ros.TransformBroadcaster()

        initialpose = rospy.get_param("~initialpose", None)
        if type(initialpose) == list:
            rospy.loginfo("using initialpose from parameter server: %s", str(initialpose))
        elif type(initialpose) == str:
            rospy.loginfo("using initialpose from script server: %s", str(initialpose))
            initialpose = rospy.get_param("/script_server/base/" + initialpose)
        else:
            rospy.loginfo("initialpose not set, using [0, 0, 0] as initialpose")
            initialpose = [0, 0, 0]

        self.initial_pose = Transform()
        self.initial_pose.translation.x = initialpose[0]
        self.initial_pose.translation.y = initialpose[1]
        quat = tf.transformations.quaternion_from_euler(0, 0, initialpose[2])
        self.initial_pose.rotation = Quaternion(*quat)

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("Emulation for laser odometry running")

    def timer_cb(self, event):
        # publish tf
        # pub odom_frame --> map
        t_loc = TransformStamped()
        t_loc.header.stamp = rospy.Time.now()
        t_loc.header.frame_id = "map"
        t_loc.child_frame_id = self.odom_frame_

        self.initial_pose = None
        t_loc.transform.translation = self.initial_pose.translation
        t_loc.transform.rotation = self.initial_pose.rotation

        transforms = [t_loc]

        self.br.sendTransform(transforms)

    def initalpose_callback(self, msg):
        rospy.loginfo("got initialpose")#:\n%s", str(msg))
        self.initial_pose.translation.x = msg.pose.pose.position.x
        self.initial_pose.translation.y = msg.pose.pose.position.y
        self.initial_pose.translation.z = msg.pose.pose.position.z

        self.initial_pose.rotation = Quaternion(*[msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,
                                                msg.pose.pose.orientation.w])



    def odometry_callback(self, msg):
        odometry = msg
        odometry.header.frame_id = self.odom_frame_
        self.odom_publisher_.publish(odometry)

if __name__ == '__main__':
    rospy.init_node('emulation_odom_laser')
    parser = argparse.ArgumentParser(conflict_handler='resolve',
                                     description="Tool for emulating nav by providing localization and move base interface")
    parser.add_argument('-o', '--odom_frame', help='odom frame name (default: \'odom_combined\')', default='odom_combined')
    args, unknown = parser.parse_known_args()
    rospy.loginfo("emulation_odom_laser running!")
    EmulationOdomLaser(args.odom_frame)
    rospy.spin()
