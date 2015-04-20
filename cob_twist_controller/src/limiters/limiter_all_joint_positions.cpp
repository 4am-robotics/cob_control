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
 *   This header contains the interface description of limiters
 *
 ****************************************************************/
#include "ros/ros.h"
#include "cob_twist_controller/limiters/limiter_all_joint_positions.h"

/**
 * Checks the positions of the joints whether the are in tolerance or not. If not the corresponding velocities vector is scaled.
 * This function multiplies the velocities that result from the IK with a limits-dependent factor in case the joint positions violate the specified tolerance.
 * The factor is calculated by using the cosine function to provide a smooth transition from 1 to zero.
 * Factor is applied on all joint velocities (although only one joint has exceeded its limits), so that the direction of the desired twist is not changed.
 * -> Important for the Use-Case to follow a trajectory exactly!
 */
KDL::JntArray LimiterAllJointPositions::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    KDL::JntArray scaled_q_dot(q_dot_ik.rows()); // according to KDL: all elements in data have 0 value; size depends on base_active (10) or not (7).
    double tolerance = this->tcParams_.tolerance / 180.0 * M_PI;


    double factor = 0.0;
    bool tolerance_surpassed = false;

    for(int i = 0; i < q_dot_ik.rows() ; ++i)
    {
        if(i < chain_.getNrOfJoints())
        {
            if((this->tcParams_.limits_max[i] - q(i)) < tolerance)    //Joint is nearer to the MAXIMUM limit
            {
                if(q_dot_ik(i) > 0)   //Joint moves towards the MAX limit
                {
                    double temp = 1.0 / pow((0.5 + 0.5 * cos(M_PI * (q(i) + tolerance - this->tcParams_.limits_max[i]) / tolerance)), 5.0);
                    factor = (temp > factor) ? temp : factor;
                    tolerance_surpassed = true;
                }
            }
            else
            {
                if((q(i) - this->tcParams_.limits_min[i]) < tolerance)    //Joint is nearer to the MINIMUM limit
                {
                    if(q_dot_ik(i) < 0)   //Joint moves towards the MIN limit
                    {
                        double temp = 1.0 / pow(0.5 + 0.5 * cos(M_PI * (q(i) - tolerance - this->tcParams_.limits_min[i]) / tolerance), 5.0);
                        factor = (temp > factor) ? temp : factor;
                        tolerance_surpassed = true;
                    }
                }
            }
        }
    }

    if (tolerance_surpassed)
    {
        ROS_WARN("Tolerance surpassed: Enforcing limits FOR ALL JOINT VELOCITIES with factor = %f", factor);
        for(int i = 0; i < q_dot_ik.rows() ; i++)
        {
            scaled_q_dot(i) = q_dot_ik(i) / factor;
        }
    }
    else
    {
        for(int i = 0; i < q_dot_ik.rows() ; i++)
        {
            scaled_q_dot(i) = q_dot_ik(i);
        }
    }

    return scaled_q_dot;
}
