#!/usr/bin/env python

import actionlib
import argparse
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

        self.odom_frame_ = odom_frame

        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initalpose_callback, queue_size=1)
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

        rospy.loginfo("Emulation for navigation running")

        # Optional move_base action
        self.move_base_mode = rospy.get_param("~move_base_mode", None)
        if self.move_base_mode == None:
            rospy.loginfo("Emulation running without move_base")
        elif self.move_base_mode == "beam" or self.move_base_mode == "linear_nav":
            self.move_base_action_name = "/move_base"
            rospy.Subscriber(self.move_base_action_name + "_simple/goal", PoseStamped, self.move_base_simple_callback, queue_size=1)
            self.as_move_base = actionlib.SimpleActionServer(self.move_base_action_name, MoveBaseAction, execute_cb=self.move_base_cb, auto_start = False)
            self.as_move_base.start()
            rospy.loginfo("Emulation running for action %s of type MoveBaseAction with mode '%s'"%(self.move_base_action_name, self.move_base_mode))
        else:
            rospy.logwarn("Emulation running without move_base due to invalid value for parameter move_base_mode: '%s'", self.move_base_mode)

    def initalpose_callback(self, msg):
        self.initial_pose.translation.x = msg.pose.pose.position.x
        self.initial_pose.translation.y = msg.pose.pose.position.y
        self.initial_pose.translation.z = msg.pose.pose.position.z

        self.initial_pose.rotation = Quaternion(*[msg.pose.pose.orientation.x,
                                                msg.pose.pose.orientation.y,
                                                msg.pose.pose.orientation.z,
                                                msg.pose.pose.orientation.w])

    def timer_cb(self, event):
        # publish tf
        # pub odom_frame --> map
        t_loc = TransformStamped()
        t_loc.header.stamp = rospy.Time.now()
        t_loc.header.frame_id = "map"
        t_loc.child_frame_id = self.odom_frame_
        t_loc.transform = self.initial_pose

        transforms = [t_loc]

        self.br.sendTransform(transforms)

    def move_base_cb(self, goal):
        pwcs = PoseWithCovarianceStamped()
        pwcs.header = goal.target_pose.header
        pwcs.pose.pose = goal.target_pose.pose
        if self.move_base_mode == "beam":
            rospy.loginfo("move_base: beaming robot to new goal")
            self.initalpose_callback(pwcs)
        elif self.move_base_mode == "linear_nav":
            move_base_linear_action_name = "/move_base_linear"
            ac_move_base_linear = actionlib.SimpleActionClient(move_base_linear_action_name, MoveBaseAction)
            rospy.loginfo("Waiting for ActionServer: %s", move_base_linear_action_name)
            if not ac_move_base_linear.wait_for_server(rospy.Duration(1)):
                rospy.logerr("Emulator move_base failed because move_base_linear action server is not available")
                self.as_move_base.set_aborted()
                return
            rospy.loginfo("send goal to %s", move_base_linear_action_name)
            ac_move_base_linear.send_goal(goal)
            ac_move_base_linear.wait_for_result()
            ac_move_base_linear_status = ac_move_base_linear.get_state()
            ac_move_base_linear_result = ac_move_base_linear.get_result()
            if ac_move_base_linear_status != GoalStatus.SUCCEEDED:
                rospy.logerr("Emulator move_base failed because move_base_linear failed")
                self.as_move_base.set_aborted()
                return
        else:
            rospy.logerr("Invalid move_base_action_mode")
            self.as_move_base.set_aborted()
            return
        rospy.loginfo("Emulator move_base succeeded")
        self.as_move_base.set_succeeded(MoveBaseResult())

    def move_base_simple_callback(self, msg):
        goal = MoveBaseGoal()
        goal.target_pose = msg
        ac_move_base = actionlib.SimpleActionClient(self.move_base_action_name, MoveBaseAction)
        rospy.loginfo("Waiting for ActionServer: %s", self.move_base_action_name)
        if not ac_move_base.wait_for_server(rospy.Duration(1)):
            rospy.logerr("Emulator move_base simple failed because move_base action server is not available")
            return
        rospy.loginfo("send goal to %s", self.move_base_action_name)
        ac_move_base.send_goal(goal)
        # ac_move_base.wait_for_result() # no need to wait for the result as this is the topic interface to move_base without feedback

if __name__ == '__main__':
    rospy.init_node('emulation_nav')
    parser = argparse.ArgumentParser(conflict_handler='resolve',
                                     description="Tool for emulating nav by providing localization and move base interface")
    parser.add_argument('-o', '--odom_frame', help='odom frame name (default: \'odom_combined\')', default='odom_combined')
    args, unknown = parser.parse_known_args()
    EmulationNav(args.odom_frame)
    rospy.spin()
