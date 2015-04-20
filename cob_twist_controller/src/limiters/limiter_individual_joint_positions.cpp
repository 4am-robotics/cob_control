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
 *   This module contains the implementation of a method to limit individual joint positions.
 *
 ****************************************************************/
#include "ros/ros.h"
#include "cob_twist_controller/limiters/limiter_individual_joint_positions.h"

/**
 * This implementation calculates limits for the joint positions without keeping the direction.
 * Then for each corresponding joint velocity an individual factor for scaling is calculated and then used.
 */
KDL::JntArray LimiterIndividualJointPositions::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    KDL::JntArray scaled_q_dot(q_dot_ik); // copy the whole q_dot array
    double tolerance = this->tcParams_.tolerance / 180.0 * M_PI;
    double factor = 0.0;

    for(uint16_t i = 0; i < scaled_q_dot.rows() ; ++i)
    {
        if(i < chain_.getNrOfJoints())
        {
            if((this->tcParams_.limits_max[i] - q(i)) < tolerance)    //Joint is nearer to the MAXIMUM limit
            {
                if(scaled_q_dot(i) > 0.0)   //Joint moves towards the MAX limit
                {
                    factor = 1.0 / pow((0.5 + 0.5 * cos(M_PI * (q(i) + tolerance - this->tcParams_.limits_max[i]) / tolerance)), 5.0);
                    scaled_q_dot(i) = scaled_q_dot(i) / factor;
                }
            }
            else
            {
                if((q(i) - this->tcParams_.limits_min[i]) < tolerance)    //Joint is nearer to the MINIMUM limit
                {
                    if(scaled_q_dot(i) < 0.0)   //Joint moves towards the MIN limit
                    {
                        factor = 1.0 / pow(0.5 + 0.5 * cos(M_PI * (q(i) - tolerance - this->tcParams_.limits_min[i]) / tolerance), 5.0);
                        scaled_q_dot(i) = scaled_q_dot(i) / factor;
                    }
                }
            }
        }
    }

    return scaled_q_dot;
}
