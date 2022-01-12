#!/usr/bin/env python

import actionlib
import argparse
import numpy
import rospy
import tf
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Transform, TransformStamped, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

class EmulationNav(object):
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

        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)
        self._transform_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initalpose_callback, queue_size=1)
        initialpose = rospy.get_param("~initialpose", None)
        if type(initialpose) == list:
            rospy.loginfo("using initialpose from parameter server: %s", str(initialpose))
        elif type(initialpose) == str:
            rospy.loginfo("using initialpose from script server: %s", str(initialpose))
            initialpose = rospy.get_param("/script_server/base/" + initialpose)
        else:
            rospy.loginfo("initialpose not set, using [0, 0, 0] as initialpose")
            initialpose = [0, 0, 0]

        self._odom_transform = Transform()
        self._odom_transform.translation.x = initialpose[0]
        self._odom_transform.translation.y = initialpose[1]
        quat = tf.transformations.quaternion_from_euler(0, 0, initialpose[2])
        self._odom_transform.rotation = Quaternion(*quat)

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        rospy.loginfo("Emulation for navigation running")

        # Optional move_base action
        self._move_base_mode = rospy.get_param("~move_base_mode", None)
        if self._move_base_mode == None:
            rospy.loginfo("Emulation running without move_base")
        elif self._move_base_mode == "beam" or self._move_base_mode == "linear_nav":
            self._move_base_action_name = "/move_base"
            rospy.Subscriber(self._move_base_action_name + "_simple/goal", PoseStamped, self.move_base_simple_callback, queue_size=1)
            self._as_move_base = actionlib.SimpleActionServer(self._move_base_action_name, MoveBaseAction, execute_cb=self.move_base_cb, auto_start = False)
            self._as_move_base.start()
            rospy.loginfo("Emulation running for action %s of type MoveBaseAction with mode '%s'"%(self._move_base_action_name, self._move_base_mode))
        else:
            rospy.logwarn("Emulation running without move_base due to invalid value for parameter move_base_mode: '%s'", self._move_base_mode)

    def initalpose_callback(self, msg):
        rospy.loginfo("Got initialpose, updating %s transformation.", self._odom_frame)
        try:
            # get pose of base_footprint within odom frame
            base_in_odom = self._buffer.lookup_transform("base_footprint", self._odom_frame, rospy.Time(0))
            # convert initialpose to matrix (necessary for matrix multiplication)
            trans_ini = tf.transformations.translation_matrix([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            rot_ini   = tf.transformations.quaternion_matrix([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            matrix_ini = numpy.dot(trans_ini, rot_ini)
            # convert footprint->odom transform to matrix (necessary for matrix multiplication)
            trans_base_odom = tf.transformations.translation_matrix([base_in_odom.transform.translation.x, base_in_odom.transform.translation.y, base_in_odom.transform.translation.z])
            rot_base_odom    = tf.transformations.quaternion_matrix([base_in_odom.transform.rotation.x, base_in_odom.transform.rotation.y, base_in_odom.transform.rotation.z, base_in_odom.transform.rotation.w])
            matrix_base_odom = numpy.dot(trans_base_odom, rot_base_odom)

            # matrix multiplication of footprint->odom transform in respect of initialpose
            matrix_new_odom = numpy.dot(matrix_ini, matrix_base_odom)

            # translate back into Transform
            trans_new_odom = tf.transformations.translation_from_matrix(matrix_new_odom)
            rot_new_odom = tf.transformations.quaternion_from_matrix(matrix_new_odom)
            self._odom_transform.translation.x = trans_new_odom[0]
            self._odom_transform.translation.y = trans_new_odom[1]
            self._odom_transform.translation.z = trans_new_odom[2]
            self._odom_transform.rotation.x = rot_new_odom[0]
            self._odom_transform.rotation.y = rot_new_odom[1]
            self._odom_transform.rotation.z = rot_new_odom[2]
            self._odom_transform.rotation.w = rot_new_odom[3]

        except Exception as e:
            rospy.logerr("Could not calculate new odom transformation:\n%s", e)
            return

    def timer_cb(self, event):
        # publish tf
        # pub odom_frame --> map
        t_loc = TransformStamped()
        t_loc.header.stamp = rospy.Time.now()
        t_loc.header.frame_id = "map"
        t_loc.child_frame_id = self._odom_frame
        t_loc.transform = self._odom_transform

        transforms = [t_loc]

        self._transform_broadcaster.sendTransform(transforms)

    def move_base_cb(self, goal):
        pwcs = PoseWithCovarianceStamped()
        pwcs.header = goal.target_pose.header
        pwcs.pose.pose = goal.target_pose.pose
        if self._move_base_mode == "beam":
            rospy.loginfo("move_base: beaming robot to new goal")
            self.initalpose_callback(pwcs)
        elif self._move_base_mode == "linear_nav":
            move_base_linear_action_name = "/move_base_linear"
            ac_move_base_linear = actionlib.SimpleActionClient(move_base_linear_action_name, MoveBaseAction)
            rospy.loginfo("Waiting for ActionServer: %s", move_base_linear_action_name)
            if not ac_move_base_linear.wait_for_server(rospy.Duration(1)):
                rospy.logerr("Emulator move_base failed because move_base_linear action server is not available")
                self._as_move_base.set_aborted()
                return
            rospy.loginfo("send goal to %s", move_base_linear_action_name)
            ac_move_base_linear.send_goal(goal)
            ac_move_base_linear.wait_for_result()
            ac_move_base_linear_status = ac_move_base_linear.get_state()
            ac_move_base_linear_result = ac_move_base_linear.get_result()
            if ac_move_base_linear_status != GoalStatus.SUCCEEDED:
                rospy.logerr("Emulator move_base failed because move_base_linear failed")
                self._as_move_base.set_aborted()
                return
        else:
            rospy.logerr("Invalid move_base_action_mode")
            self._as_move_base.set_aborted()
            return
        rospy.loginfo("Emulator move_base succeeded")
        self._as_move_base.set_succeeded(MoveBaseResult())

    def move_base_simple_callback(self, msg):
        goal = MoveBaseGoal()
        goal.target_pose = msg
        ac_move_base = actionlib.SimpleActionClient(self._move_base_action_name, MoveBaseAction)
        rospy.loginfo("Waiting for ActionServer: %s", self._move_base_action_name)
        if not ac_move_base.wait_for_server(rospy.Duration(1)):
            rospy.logerr("Emulator move_base simple failed because move_base action server is not available")
            return
        rospy.loginfo("send goal to %s", self._move_base_action_name)
        ac_move_base.send_goal(goal)
        # ac_move_base.wait_for_result() # no need to wait for the result as this is the topic interface to move_base without feedback

if __name__ == '__main__':
    rospy.init_node('emulation_nav')
    parser = argparse.ArgumentParser(conflict_handler='resolve',
                                     description="Tool for emulating nav by providing localization and move base interface")
    parser.add_argument('-o', '--odom_frame', help='odom frame name (default: \'odom_combined\')', default='odom_combined')
    args, unknown = parser.parse_known_args()
    rospy.loginfo("emulation_nav started!")
    EmulationNav(args.odom_frame)
    rospy.spin()
