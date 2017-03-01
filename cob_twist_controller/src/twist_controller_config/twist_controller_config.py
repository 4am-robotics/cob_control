#!/usr/bin/python
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
 *   Implementation of a dynamic_reconfigure client for twist_controller.
 *
"""
import rospy
from dynamic_reconfigure.client import Client
from cob_twist_controller.cfg.TwistControllerConfig import *

'''
Available keys for the dynamic_reconfigure update call
'''
NUM_FILT = 'numerical_filtering'
DAMP_METHOD = 'damping_method'
DAMP_FACT = 'damping_factor'
LAMBDA_MAX = 'lambda_max'
W_THRESH = 'w_threshold'
SLOPE_DAMPING='slope_damping'
BETA = 'beta'
EPS_DAMP = 'eps_damping'
EPS_TRUNC = 'eps_truncation'

SOLVER = 'solver'
PRIO = 'priority'
K_H = 'k_H'

CONSTR_JLA = 'constraint_jla'
PRIO_JLA = 'priority_jla'
K_H_JLA = 'k_H_jla'
ACTIV_THRESH_JLA = 'activation_threshold_jla'
ACTIV_BUF_JLA = 'activation_buffer_jla'
ACTIV_POS_THRESH_JLA = 'activation_position_threshold_jla'
ACTIV_SPEED_THRESH_JLA = 'activation_speed_threshold_jla'
CRIT_THRESH_JLA = 'critical_threshold_jla'
DAMP_JLA = 'damping_jla'
DAMP_SPEED_JLA = 'damping_speed_jla'

CONSTR_CA = 'constraint_ca'
PRIO_CA = 'priority_ca'
K_H_CA = 'k_H_ca'
ACTIV_THRESH_CA = 'activation_threshold_ca'
ACTIV_BUF_CA = 'activation_buffer_ca'
CRIT_THRESH_CA = 'critical_threshold_ca'
DAMP_CA = 'damping_ca'

KEEP_DIR = 'keep_direction'
ENF_POS_LIM = 'enforce_pos_limits'
ENF_VEL_LIM = 'enforce_vel_limits'
ENF_ACC_LIM = 'enforce_acc_limits'
TOL = 'limits_tolerance'
MAX_VEL_LIN_BASE = 'max_vel_lin_base'
MAX_VEL_ROT_BASE = 'max_vel_rot_base'

KIN_EXT = 'kinematic_extension'
EXT_RATIO = 'extension_ratio'

'''
Class inherits from dynamic_reconfigure.client.Client and implements some wrapper methods
'''
class TwistControllerReconfigureClient(Client):

    def __init__(self, timeout = None):
        super(TwistControllerReconfigureClient, self).__init__('twist_controller', timeout)
        self._current_config = {}
        self._update_config = {}

    def init(self):
        self._current_config = self.get_configuration()
        self._update_config.clear()

    def set_config_param(self, cfg_key, cfg_value):
        if cfg_key in self._current_config:
            self._update_config[cfg_key] = cfg_value
        else:
            rospy.logerr('Cannot update config with key {0}! Not available in current config.'.format(cfg_key))

    def update(self):
        self.update_configuration(self._update_config)
        self._update_config.clear()
