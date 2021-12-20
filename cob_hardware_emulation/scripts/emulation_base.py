#!/usr/bin/env python

import argparse
import copy
import math

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class EmulationBase(object):
    def __init__(self, odom_frame):
        # this node emulates the controllers of a base including twist_controller and odometry_controller
        #
        # interfaces
        # - subscribers:
        #   - /base/twist_controller/command [geometry_msgs/Twist]
        # - publishers:
        #   - /base/odometry_controller/odometry [nav_msgs/Odometry]
        #   - tf (odom_frame --> base_footprint)

        # TODO
        # - speed factor

        self.odom_frame_ = odom_frame

        rospy.Subscriber("/base/twist_controller/command", Twist, self.twist_callback, queue_size=1)
        self.pub_odom = rospy.Publisher("/base/odometry_controller/odometry", Odometry, queue_size=1)
        rospy.Service("/base/odometry_controller/reset_odometry", Trigger, self.reset_odometry)
        self.br = tf2_ros.TransformBroadcaster()

        self.timestamp_last_update = rospy.Time.now()

        self.twist = Twist()
        self.timestamp_last_twist = rospy.Time(0)

        self.odom = Odometry()
        self.odom.header.frame_id = self.odom_frame_
        self.odom.child_frame_id = "base_footprint"
        self.odom.pose.pose.orientation.w = 1 # initialize orientation with a valid quaternion

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("Emulation for base running")

    def reset_odometry(self, req):
        self.odom.pose.pose = Pose()
        self.odom.pose.pose.orientation.w = 1

        return TriggerResponse(True, "odometry resetted")

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
        # pub base_footprint --> odom_frame
        t_odom = TransformStamped()
        t_odom.header.stamp = rospy.Time.now()
        t_odom.header.frame_id = self.odom_frame_
        t_odom.child_frame_id = "base_footprint"
        t_odom.transform.translation = self.odom.pose.pose.position
        t_odom.transform.rotation = self.odom.pose.pose.orientation

        transforms = [t_odom]

        self.br.sendTransform(transforms)

if __name__ == '__main__':
    rospy.init_node('emulation_base')
    parser = argparse.ArgumentParser(conflict_handler='resolve',
                                     description="Tool for emulating base by publishing odometry and propagating base_footprint.")
    parser.add_argument('-o', '--odom_frame', help='odom frame name (default: \'odom_combined\')', default='odom_combined')
    args, unknown = parser.parse_known_args()
    EmulationBase(args.odom_frame)
    rospy.spin()
