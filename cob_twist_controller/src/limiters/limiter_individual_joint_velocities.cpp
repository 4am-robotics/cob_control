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
 *   This module contains the implementation of the method to limit joint velocities individually.
 *
 ****************************************************************/
#include "ros/ros.h"
#include "cob_twist_controller/limiters/limiter_individual_joint_velocities.h"

/**
 * This implementation calculates limits for the joint velocities without keeping the direction.
 * For each joint velocity in the vector an individual factor for scaling is calculated and used.
 */
KDL::JntArray LimiterIndividualJointVelocities::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    KDL::JntArray q_dot_norm(q_dot_ik);
    double max_factor = 1.0;

    uint16_t maxDof = this->tcParams_.dof;
    std::vector<double> tmpLimits = this->tcParams_.limits_vel;
    if(this->tcParams_.base_active)
    {
        maxDof += 3; // additional 3 DOF for the base (X, Y, Z)
        tmpLimits.push_back(this->tcParams_.max_vel_lin_base); // BaseTransX limit
        tmpLimits.push_back(this->tcParams_.max_vel_lin_base); // BaseTransY limit
        tmpLimits.push_back(this->tcParams_.max_vel_rot_base); // BaseRotZ limit
    }

    for(uint16_t i=0; i < maxDof; ++i)
    {
        max_factor = 1.0;
        if(max_factor < std::fabs((q_dot_ik(i) / tmpLimits[i])))
        {
            max_factor = std::fabs((q_dot_ik(i) / tmpLimits[i]));
            q_dot_norm(i) = q_dot_ik(i) / max_factor;
            //ROS_WARN("Joint %d exceeds limit: Desired %f, Limit %f, Factor %f", i, q_dot_ik(i), limits_vel_[i], max_factor);
        }
    }

    return q_dot_norm;
}
