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
 *   This header contains the description of a limiter for all joint velocities
 *
 ****************************************************************/
#include "ros/ros.h"
#include "cob_twist_controller/limiters/limiter_all_joint_velocities.h"

/**
 * Enforce limits on all joint velocities to keep direction (former known as normalize_velocities).
 * Limits all velocities according to the limits_vel vector if necessary.
 */
KDL::JntArray LimiterAllJointVelocities::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    KDL::JntArray q_dot_norm(q_dot_ik);
    double max_factor = 1.0;
    for(unsigned int i=0; i < this->tcParams_.dof; i++)
    {
        if(max_factor < std::fabs((q_dot_ik(i)/this->tcParams_.limits_vel[i])))
        {
            max_factor = std::fabs((q_dot_ik(i)/this->tcParams_.limits_vel[i]));
            //ROS_WARN("Joint %d exceeds limit: Desired %f, Limit %f, Factor %f", i, q_dot_ik(i), limits_vel_[i], max_factor);
        }
    }

    if(this->tcParams_.base_active)
    {
        if(max_factor < std::fabs((q_dot_ik(this->tcParams_.dof)/this->tcParams_.max_vel_lin_base)))
        {
            max_factor = std::fabs((q_dot_ik(this->tcParams_.dof) / this->tcParams_.max_vel_lin_base));
            //ROS_WARN("BaseTransX exceeds limit: Desired %f, Limit %f, Factor %f", q_dot_ik(dof_), max_vel_lin_base_, max_factor);
        }
        if(max_factor < std::fabs((q_dot_ik(this->tcParams_.dof + 1) / this->tcParams_.max_vel_lin_base)))
        {
            max_factor = std::fabs((q_dot_ik(this->tcParams_.dof + 1) / this->tcParams_.max_vel_lin_base));
            //ROS_WARN("BaseTransY exceeds limit: Desired %f, Limit %f, Factor %f", q_dot_ik(dof_+1), max_vel_lin_base_, max_factor);
        }
        if(max_factor < std::fabs((q_dot_ik(this->tcParams_.dof + 2) / this->tcParams_.max_vel_rot_base)))
        {
            max_factor = std::fabs((q_dot_ik(this->tcParams_.dof + 2) / this->tcParams_.max_vel_rot_base));
            //ROS_WARN("BaseRotZ exceeds limit: Desired %f, Limit %f, Factor %f", q_dot_ik(dof_+2), max_vel_rot_base_, max_factor);
        }
    }

    if(max_factor > 1.0)
    {
        ROS_WARN("Normalizing velocities (Factor: %f!", max_factor);
        for(uint8_t i = 0; i < this->tcParams_.dof; ++i)
        {
            q_dot_norm(i) = q_dot_ik(i) / max_factor;
            //ROS_WARN("Joint %d Normalized %f", i, q_dot_norm(i));
        }

        if(this->tcParams_.base_active)
        {
            q_dot_norm(this->tcParams_.dof) = q_dot_ik(this->tcParams_.dof) / max_factor;
            q_dot_norm(this->tcParams_.dof + 1) = q_dot_ik(this->tcParams_.dof + 1) / max_factor;
            q_dot_norm(this->tcParams_.dof + 2) = q_dot_ik(this->tcParams_.dof + 2) / max_factor;
        }
    }

    return q_dot_norm;


}
