/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
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
 * \date Date of creation: April, 2015
 *
 * \brief
 *   Different data types for CobTwistController to be used in other modules
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_DATA_TYPES_H_
#define COB_TWIST_CONTROLLER_DATA_TYPES_H_

#include <vector>
#include <stdint.h>

enum InterfaceType {
    VELOCITY = 0,
    POSITION = 1,
};


struct TwistControllerParams {
    uint8_t dof;
    bool base_active;
    bool base_compensation;
    double max_vel_lin;
    double max_vel_rot;
    double max_vel_lin_base;
    double max_vel_rot_base;
    double tolerance;

    bool keep_direction;
    bool enforce_pos_limits;
    bool enforce_vel_limits;

    InterfaceType interface_type;


    // added limits from URDF file
    std::vector<double> limits_vel;
    std::vector<double> limits_min;
    std::vector<double> limits_max;
};

#endif /* COB_TWIST_CONTROLLER_DATA_TYPES_H_ */
