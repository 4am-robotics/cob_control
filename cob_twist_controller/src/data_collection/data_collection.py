#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time
import csv
import datetime
import rospy

import abc

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cob_control_msgs.msg import ObstacleDistances
import tf

DEL = ','
QUOTE = '"'
DATE_TIME_FMT = '%d.%m.%Y %H:%M:%S.%f'  # should not consist the DEL char!!!


class DataKraken(object):

    __metaclass__ = abc.ABCMeta

    def __init__(self, file_path, topic_name, data_class):
        self.file_path_ = file_path
        self.csv_writer_ = None
        self.topic_name_ = topic_name
        self.data_class_ = data_class
        self.data_rows_ = []
        self.subscriber_ = None
        self.start_ = None

    def open(self):
        success = False
        try:
            with open(self.file_path_, 'wt') as csvfile:
                self.csv_writer_ = csv.writer(
                    csvfile, delimiter = DEL, quotechar = QUOTE)
                success = self.csv_writer_ is not None
            if success:
                self.subscriber_ = rospy.Subscriber(
                    self.topic_name_, self.data_class_, self.callback)
        except IOError as ioe:
            rospy.logerr(str(ioe))
        return success

    def writeAllData(self):
        success = False
        if len(self.data_rows_) > 0:
            with open(self.file_path_, 'wt') as csvfile:
                self.csv_writer_ = csv.writer(
                    csvfile, delimiter = DEL, quotechar = QUOTE)
                self.csv_writer_.writerows(self.data_rows_)
                success = True
                rospy.loginfo(
                    self.__class__.__name__ + ': Successfully wrote data into file: ' + self.file_path_)
        else:
            rospy.logerr(self.__class__.__name__ +
                         ': Could not find any data to write into file: ' + self.file_path_)
        return success

    @abc.abstractmethod
    def callback(self, data):
        raise NotImplementedError


class JointStateDataKraken(DataKraken):

    def __init__(self, file_path):
        super(JointStateDataKraken, self).__init__(
            file_path, 'joint_states', JointState)
        self.init_ = True
        self.jointstate_sub_ = None

    def callback(self, data):
        if self.init_:
            self.start_ = datetime.datetime.now()
            header = []
            header.append('time stamp')
            header.append('time delta')
            for name in data.name:
                header.append(name + '_position')
            for name in data.name:
                header.append(name + '_velocity')
            self.data_rows_.append(header)
            self.init_ = False

        now = datetime.datetime.now()
        delta = now - self.start_

        data_row = []
        data_row.append(now.strftime(DATE_TIME_FMT))
        data_row.append('%.3f' % delta.total_seconds())
        data_row.extend(list(data.position))
        data_row.extend(list(data.velocity))
        self.data_rows_.append(data_row)


class ObstacleDistanceDataKraken(DataKraken):

    def __init__(self, file_path):
        super(ObstacleDistanceDataKraken, self).__init__(
            file_path, 'obstacle_distance', ObstacleDistances)
        self.init_ = True

    def callback(self, data):
        if self.init_:
            self.start_ = datetime.datetime.now()
            header = []
            header.append('time stamp')
            header.append('time delta')
            header.append('link_of_interest')
            header.append('obstacle')
            header.append('distance')
            header.append('frame_vector_x')
            header.append('frame_vector_y')
            header.append('frame_vector_z')
            header.append('np_frame_vector_x')
            header.append('np_frame_vector_y')
            header.append('np_frame_vector_z')
            header.append('np_obstacle_vector_x')
            header.append('np_obstacle_vector_y')
            header.append('np_obstacle_vector_z')
            self.data_rows_.append(header)
            self.init_ = False

        now = datetime.datetime.now()
        delta = now - self.start_
        for elem in data.distances:
            data_row = [now.strftime(DATE_TIME_FMT), '%.3f' % delta.total_seconds()]
            data_row.append(elem.link_of_interest)
            data_row.append(elem.obstacle_id)
            data_row.append(elem.distance)
            data_row.append(elem.frame_vector.x)
            data_row.append(elem.frame_vector.y)
            data_row.append(elem.frame_vector.z)
            data_row.append(elem.nearest_point_frame_vector.x)
            data_row.append(elem.nearest_point_frame_vector.y)
            data_row.append(elem.nearest_point_frame_vector.z)
            data_row.append(elem.nearest_point_obstacle_vector.x)
            data_row.append(elem.nearest_point_obstacle_vector.y)
            data_row.append(elem.nearest_point_obstacle_vector.z)
            self.data_rows_.append(data_row)


class TwistDataKraken(DataKraken):

    def __init__(self, file_path, for_base = False, use_twist_stamped = True):
        self.use_twist_stamped_ = use_twist_stamped

        if for_base:
            super(TwistDataKraken, self).__init__(
                file_path, '/base/twist_controller/command', Twist)
        else:
            if self.use_twist_stamped_:
                super(TwistDataKraken, self).__init__(
                    file_path, 'twist_controller/command_twist_stamped', TwistStamped)
            else:
                super(TwistDataKraken, self).__init__(
                    file_path, 'twist_controller/command_twist', Twist)
        self.init_ = True

    def callback(self, data):
        if self.use_twist_stamped_:
            frame_id = data.header.frame_id + '_'
            twist_data = data.twist
        else:
            frame_id = ''
            twist_data = data

        if self.init_:
            self.start_ = datetime.datetime.now()
            header = []

            header.append(frame_id + 'time stamp')
            header.append(frame_id + 'time delta')
            header.append(frame_id + 'linear_x')
            header.append(frame_id + 'linear_y')
            header.append(frame_id + 'linear_z')
            header.append(frame_id + 'angular_x')
            header.append(frame_id + 'angular_y')
            header.append(frame_id + 'angular_z')
            self.data_rows_.append(header)
            self.init_ = False
        data_row = []

        now = datetime.datetime.now()
        delta = now - self.start_
        data_row.append(now.strftime(DATE_TIME_FMT))
        data_row.append('%.3f' % delta.total_seconds())
        data_row.append(twist_data.linear.x)
        data_row.append(twist_data.linear.y)
        data_row.append(twist_data.linear.z)
        data_row.append(twist_data.angular.x)
        data_row.append(twist_data.angular.y)
        data_row.append(twist_data.angular.z)
        self.data_rows_.append(data_row)


class JointVelocityDataKraken(DataKraken):

    def __init__(self, file_path):
        super(JointVelocityDataKraken, self).__init__(
            file_path, 'joint_group_velocity_controller/command', Float64MultiArray)
        self.init_ = True

    def callback(self, data):
        if self.init_:
            self.start_ = datetime.datetime.now()
            header = []
            header.append('time stamp')
            header.append('time delta')
            for idx, jointvel in enumerate(data.data):
                header.append(str(idx + 1))
            self.data_rows_.append(header)
            self.init_ = False
        data_row = []
        now = datetime.datetime.now()
        delta = now - self.start_
        data_row.append(now.strftime(DATE_TIME_FMT))
        data_row.append('%.3f' % delta.total_seconds())

        for jointvel in data.data:
            data_row.append(jointvel)
        self.data_rows_.append(data_row)


class FrameTrackingDataKraken(DataKraken):

    def __init__(self, file_path, root_frame, chain_tip_link, tracking_frame):
        super(FrameTrackingDataKraken, self).__init__(
            file_path, None, None)
        self.init_ = True
        self.timed_transform_ = None
        self.tf_ = None
        self.root_frame_ = root_frame
        self.chain_tip_link_ = chain_tip_link
        self.tracking_frame_ = tracking_frame

    def open(self):
        success = False
        try:
            with open(self.file_path_, 'wt') as csvfile:
                self.csv_writer_ = csv.writer(
                    csvfile, delimiter = DEL, quotechar = QUOTE)
                success = self.csv_writer_ is not None
            if success:
                self.tf_ = tf.TransformListener()
                self.timed_transform_ = rospy.Timer(rospy.Duration.from_sec(0.1), self.callback)
        except IOError as ioe:
            rospy.logerr(str(ioe))
        return success

    def writeAllData(self):
        self.timed_transform_.shutdown()
        self.timed_transform_.join(5.0)
        return super(FrameTrackingDataKraken, self).writeAllData()

    def callback(self, data):
        if self.init_:
            self.start_ = datetime.datetime.now()
            header = []
            header.append('time stamp')
            header.append('time delta')
            for frame in (self.chain_tip_link_, self.tracking_frame_):
                header.append('x Trans ' + self.root_frame_ + ' to ' + frame)
                header.append('y Trans ' + self.root_frame_ + ' to ' + frame)
                header.append('z Trans ' + self.root_frame_ + ' to ' + frame)
                header.append('x Quat ' + self.root_frame_ + ' to ' + frame)
                header.append('y Quat ' + self.root_frame_ + ' to ' + frame)
                header.append('z Quat ' + self.root_frame_ + ' to ' + frame)
                header.append('w Quat ' + self.root_frame_ + ' to ' + frame)
                header.append('roll ' + self.root_frame_ + ' to ' + frame)
                header.append('pitch ' + self.root_frame_ + ' to ' + frame)
                header.append('yaw ' + self.root_frame_ + ' to ' + frame)
            self.data_rows_.append(header)
            self.init_ = False

        trans_root_tip, quat_root_tip, rpy_root_tip = None, None, None
        try:
            # tf_time = t = self.tf_.getLatestCommonTime(self.root_frame_, self.tracking_frame_)
            (trans_root_tip, quat_root_tip) = self.tf_.lookupTransform(self.root_frame_, self.chain_tip_link_, rospy.Time(0))
            rpy_root_tip = tf.transformations.euler_from_quaternion(quat_root_tip)
        except Exception as e:
            print('Exception chain_tip_link_: ' + str(e))

        trans_root_track, quat_root_track, rpy_root_track = None, None, None
        try:
            # tf_time = t = self.tf_.getLatestCommonTime(self.root_frame_, self.tracking_frame_)
            # if self.tf_.waitForTransform(self.root_frame_, self.tracking_frame_, tf_time, rospy.Duration(0.5)):
            (trans_root_track, quat_root_track) = self.tf_.lookupTransform(self.root_frame_, self.tracking_frame_, rospy.Time(0))
            rpy_root_track = tf.transformations.euler_from_quaternion(quat_root_track)

        except Exception as e:
            print('Exception tracking_frame_: ' + str(e))


        if rpy_root_tip is not None and rpy_root_track is not None:
            data_row = []

            now = datetime.datetime.now()
            delta = now - self.start_
            data_row.append(now.strftime(DATE_TIME_FMT))
            data_row.append('%.3f' % delta.total_seconds())

            data_row.extend(trans_root_tip)
            data_row.extend(quat_root_tip)
            data_row.extend(rpy_root_tip)

            data_row.extend(trans_root_track)
            data_row.extend(quat_root_track)
            data_row.extend(rpy_root_track)

            self.data_rows_.append(data_row)


class OdometryDataKraken(DataKraken):

    def __init__(self, file_path):
        super(OdometryDataKraken, self).__init__(
            file_path, '/base/odometry_controller/odometry', Odometry)
        self.init_ = True

    def callback(self, data):
        if self.init_:
            self.start_ = datetime.datetime.now()
            header = []
            header.append(data.header.frame_id + '_time stamp')
            header.append(data.header.frame_id + '_time delta')
            header.append(data.header.frame_id + '_pos_x')
            header.append(data.header.frame_id + '_pos_y')
            header.append(data.header.frame_id + '_pos_z')
            header.append(data.header.frame_id + '_quat_x')
            header.append(data.header.frame_id + '_quat_y')
            header.append(data.header.frame_id + '_quat_z')
            header.append(data.header.frame_id + '_quat_w')
            header.append(data.header.frame_id + '_roll')
            header.append(data.header.frame_id + '_pitch')
            header.append(data.header.frame_id + '_yaw')
            header.append(data.header.frame_id + '_twist_linear_x')
            header.append(data.header.frame_id + '_twist_linear_y')
            header.append(data.header.frame_id + '_twist_linear_z')
            header.append(data.header.frame_id + '_twist_angular_x')
            header.append(data.header.frame_id + '_twist_angular_y')
            header.append(data.header.frame_id + '_twist_angular_z')
            self.data_rows_.append(header)
            self.init_ = False
        data_row = []

        now = datetime.datetime.now()
        delta = now - self.start_
        data_row.append(now.strftime(DATE_TIME_FMT))
        data_row.append('%.3f' % delta.total_seconds())

        rpy = tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w,))
        data_row.append(data.pose.pose.position.x)
        data_row.append(data.pose.pose.position.y)
        data_row.append(data.pose.pose.position.z)
        data_row.append(data.pose.pose.orientation.x)
        data_row.append(data.pose.pose.orientation.y)
        data_row.append(data.pose.pose.orientation.z)
        data_row.append(data.pose.pose.orientation.w)
        data_row.append(rpy[0])
        data_row.append(rpy[1])
        data_row.append(rpy[2])
        data_row.append(data.twist.twist.linear.x)
        data_row.append(data.twist.twist.linear.y)
        data_row.append(data.twist.twist.linear.z)
        data_row.append(data.twist.twist.angular.x)
        data_row.append(data.twist.twist.angular.y)
        data_row.append(data.twist.twist.angular.z)
        self.data_rows_.append(data_row)


