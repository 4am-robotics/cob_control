#!/usr/bin/env python
"""
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: July, 2015
 *
 * \brief
 *   Implementation of special data kraken classes to subscribe to several topics. 
 *   In callback methods data is collected, put into a list representing rows. 
 *   Then the rows can be written to a csv file.  
 *
"""


import time
import csv
import datetime
import rospy

import abc

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from cob_obstacle_distance.msg import ObstacleDistances

DEL = ','
QUOTE = '"'
DATE_TIME_FMT = '%d.%m.%Y %H:%M:%S.%f' # should not consist the DEL char!!!

class DataKraken(object):
    
    __metaclass__ = abc.ABCMeta
    
    def __init__(self, file_path, topic_name, data_class):
        self.file_path_ = file_path
        self.csv_writer_ = None
        self.topic_name_ = topic_name
        self.data_class_ = data_class
        self.data_rows_ = []
        self.subscriber_ = None
        
    def open(self):
        success = False
        try: 
            with open(self.file_path_, 'wt') as csvfile: 
                self.csv_writer_ = csv.writer(csvfile, delimiter=DEL, quotechar=QUOTE)
                success = self.csv_writer_ is not None                
            if success: 
                self.subscriber_ = rospy.Subscriber(self.topic_name_, self.data_class_, self.callback)
        except IOError as ioe:
            rospy.logerr(str(ioe)) 
        return success
    
    def writeAllData(self):
        success = False
        if len(self.data_rows_) > 0: 
            with open(self.file_path_, 'wt') as csvfile:
                self.csv_writer_ = csv.writer(csvfile, delimiter=DEL, quotechar=QUOTE) 
                self.csv_writer_.writerows(self.data_rows_)
                success = True
                rospy.loginfo(self.__class__.__name__ + ': Successfully wrote data into file: ' + self.file_path_)
        else:
            rospy.logerr(self.__class__.__name__ + ': Could not find any data to write into file: ' + self.file_path_)
        return success
    
    @abc.abstractmethod
    def callback(self, data):
        raise NotImplementedError
         

class JointStateDataKraken(DataKraken):
    
    def __init__(self, file_path):
        super(JointStateDataKraken, self).__init__(file_path, 'joint_states', JointState)
        self.init_ = True
        self.jointstate_sub_ = None
           
    def callback(self, data):
        if self.init_: 
            header = []
            header.append('time')
            for name in data.name: 
                header.append(name + '_position')
            for name in data.name: 
                header.append(name + '_velocity')
            self.data_rows_.append(header)
            self.init_ = False
            
        data_row = []
        data_row.append(datetime.datetime.now().strftime(DATE_TIME_FMT))
        data_row.extend(list(data.position))
        data_row.extend(list(data.velocity))
        self.data_rows_.append(data_row)


class ObstacleDistanceDataKraken(DataKraken):
    
    def __init__(self, file_path):
        super(ObstacleDistanceDataKraken, self).__init__(file_path, 'obstacle_distance', ObstacleDistances)
        self.init_ = True
        
    def callback(self, data):
        if self.init_: 
            header = []
            header.append('time')
            for elem in data.distances:
                header.append(elem.header.frame_id + '_distance')
                header.append(elem.header.frame_id + '_distance_vector_x')
                header.append(elem.header.frame_id + '_distance_vector_y')
                header.append(elem.header.frame_id + '_distance_vector_z')
                header.append(elem.header.frame_id + '_collision_pnt_vector_x')
                header.append(elem.header.frame_id + '_collision_pnt_vector_y')
                header.append(elem.header.frame_id + '_collision_pnt_vector_z')
            self.data_rows_.append(header)
            self.init_ = False
        
        data_row = []
        data_row.append(datetime.datetime.now().strftime(DATE_TIME_FMT))
        for elem in data.distances:
            data_row.append(elem.distance)
            data_row.append(elem.distance_vector.x)
            data_row.append(elem.distance_vector.y)
            data_row.append(elem.distance_vector.z)
            data_row.append(elem.collision_pnt_vector.x)
            data_row.append(elem.collision_pnt_vector.y)
            data_row.append(elem.collision_pnt_vector.z)
        self.data_rows_.append(data_row)


class TwistDataKraken(DataKraken):
    
    def __init__(self, file_path):
        super(TwistDataKraken, self).__init__(file_path, 'twist_controller/command_twist_stamped', TwistStamped)
        self.init_ = True
    
    def callback(self, data):
        if self.init_: 
            header = []
            header.append(data.header.frame_id + '_time')
            header.append(data.header.frame_id + '_linear_x')
            header.append(data.header.frame_id + '_linear_y')
            header.append(data.header.frame_id + '_linear_z')
            header.append(data.header.frame_id + '_angular_x')
            header.append(data.header.frame_id + '_angular_y')
            header.append(data.header.frame_id + '_angular_z')            
            self.data_rows_.append(header)
            self.init_ = False
        data_row = []
        data_row.append(datetime.datetime.now().strftime(DATE_TIME_FMT))
        data_row.append(data.twist.linear.x)
        data_row.append(data.twist.linear.y)
        data_row.append(data.twist.linear.z)
        data_row.append(data.twist.angular.x)
        data_row.append(data.twist.angular.y)
        data_row.append(data.twist.angular.z)
        self.data_rows_.append(data_row)
        

class JointVelocityDataKraken(DataKraken):
    
    def __init__(self, file_path):
        super(JointVelocityDataKraken, self).__init__(file_path, 'joint_group_velocity_controller/command', Float64MultiArray)
        self.init_ = True
    
    def callback(self, data):
        if self.init_: 
            header = []
            header.append('time')
            for idx, jointvel in enumerate(data.data):
                header.append(str(idx + 1))           
            self.data_rows_.append(header)
            self.init_ = False
        data_row = []
        data_row.append(datetime.datetime.now().strftime(DATE_TIME_FMT))
        for jointvel in data.data:
            data_row.append(jointvel)
        self.data_rows_.append(data_row)
