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
 * \date Date of creation: March, 2015
 *
 * \brief
 *   This module contains the implementation of all classes and their
 *   methods to limit joint positions / velocities / accelerations.
 *
 ****************************************************************/
#include <vector>
#include <ros/ros.h>

#include "cob_twist_controller/limiters/limiter.h"

/* BEGIN LimiterContainer ***************************************************************************************/
/**
 * This implementation calls enforce limits on all registered Limiters in the limiters vector.
 * The method is based on the last calculation of q_dot.
 */
KDL::JntArray LimiterContainer::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    // If nothing to do just return q_dot.
    KDL::JntArray q_dot_norm(q_dot_ik);
    for (LimIter_t it = this->limiters_.begin(); it != this->limiters_.end(); it++)
    {
        q_dot_norm = (*it)->enforceLimits(q_dot_norm, q);
    }

    return q_dot_norm;
}

/**
 * Building the limiters vector according the the chosen parameters.
 */
void LimiterContainer::init()
{
    this->eraseAll();

    if (limiter_params_.keep_direction)
    {
        if (limiter_params_.enforce_pos_limits)
        {
            this->add(new LimiterAllJointPositions(limiter_params_));
        }

        if (limiter_params_.enforce_vel_limits)
        {
            this->add(new LimiterAllJointVelocities(limiter_params_));
        }

        if (limiter_params_.enforce_acc_limits)
        {
            this->add(new LimiterAllJointAccelerations(limiter_params_));
        }
    }
    else
    {
        if (limiter_params_.enforce_pos_limits)
        {
            this->add(new LimiterIndividualJointPositions(limiter_params_));
        }

        if (limiter_params_.enforce_vel_limits)
        {
            this->add(new LimiterIndividualJointVelocities(limiter_params_));
        }

        if (limiter_params_.enforce_acc_limits)
        {
            this->add(new LimiterIndividualJointAccelerations(limiter_params_));
        }
    }
}

/**
 * Deletes all limiters and clears the vector holding them.
 */
void LimiterContainer::eraseAll()
{
    for (uint32_t cnt = 0; cnt < this->limiters_.size(); ++cnt)
    {
        const LimiterBase* lb = this->limiters_[cnt];
        delete(lb);
    }

    this->limiters_.clear();
}

/**
 * Adding new limiters to the vector.
 */
void LimiterContainer::add(const LimiterBase* lb)
{
    this->limiters_.push_back(lb);
}

/**
 * Destruction of the whole container
 */
LimiterContainer::~LimiterContainer()
{
    this->eraseAll();
}
/* END LimiterContainer *****************************************************************************************/

/* BEGIN LimiterAllJointPositions *******************************************************************************/
/**
 * Checks the positions of the joints whether they are in limits_tolerance or not. If not the corresponding velocities vector is scaled.
 * This function multiplies the velocities that result from the IK with a limits-dependent factor in case the joint positions violate the specified limits_tolerance.
 * The factor is calculated by using the cosine function to provide a smooth transition from 1 to zero.
 * Factor is applied on all joint velocities (although only one joint has exceeded its limits), so that the direction of the desired twist is not changed.
 * -> Important for the Use-Case to follow a trajectory exactly!
 */
KDL::JntArray LimiterAllJointPositions::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    KDL::JntArray q_dot_norm(q_dot_ik);
    double tolerance = limiter_params_.limits_tolerance / 180.0 * M_PI;
    double max_factor = 1.0;
    int joint_index = -1;

    for (unsigned int i = 0; i < q_dot_ik.rows(); i++)
    {
        if((limiter_params_.limits_max[i] - LIMIT_SAFETY_THRESHOLD <= q(i) && q_dot_ik(i) > 0) ||
           (limiter_params_.limits_min[i] + LIMIT_SAFETY_THRESHOLD >= q(i) && q_dot_ik(i) < 0))
        {
            ROS_ERROR_STREAM("Joint " << i << " violates its limits. Setting to Zero!");
            KDL::SetToZero(q_dot_norm);
            return q_dot_norm;
        }

        if (fabs(limiter_params_.limits_max[i] - q(i)) <= tolerance)  // Joint is close to the MAXIMUM limit
        {
            if (q_dot_ik(i) > 0)  // Joint moves towards the MAX limit
            {
                double temp = 1.0 / pow((0.5 + 0.5 * cos(M_PI * (q(i) + tolerance - limiter_params_.limits_max[i]) / tolerance)), 5.0);
                // double temp = tolerance / fabs(limiter_params_.limits_max[i] - q(i));
                if (temp > max_factor)
                {
                    max_factor = temp;
                    joint_index = i;
                }
            }
        }

        if (fabs(q(i) - limiter_params_.limits_min[i]) <= tolerance)  // Joint is close to the MINIMUM limit
        {
            if (q_dot_ik(i) < 0)  // Joint moves towards the MIN limit
            {
                double temp = 1.0 / pow(0.5 + 0.5 * cos(M_PI * (q(i) - tolerance - limiter_params_.limits_min[i]) / tolerance), 5.0);
                // double temp = tolerance / fabs(q(i) - limiter_params_.limits_min[i]);
                if (temp > max_factor)
                {
                    max_factor = temp;
                    joint_index = i;
                }
            }
        }
    }

    if (max_factor > 1.0)
    {
        ROS_ERROR_STREAM_THROTTLE(1, "Position tolerance surpassed (by Joint " << joint_index << "): Scaling ALL VELOCITIES with factor = " << max_factor);
        for (unsigned int i = 0; i < q_dot_ik.rows(); i++)
        {
            q_dot_norm(i) = q_dot_ik(i) / max_factor;
        }
    }

    return q_dot_norm;
}
/* END LimiterAllJointPositions *********************************************************************************/

/* BEGIN LimiterAllJointVelocities ******************************************************************************/
/**
 * Enforce limits on all joint velocities to keep direction.
 * Limits all velocities according to the limits_vel vector if necessary.
 */
KDL::JntArray LimiterAllJointVelocities::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    KDL::JntArray q_dot_norm(q_dot_ik);
    double max_factor = 1.0;
    int joint_index = -1;

    for (unsigned int i = 0; i < q_dot_ik.rows(); i++)
    {
        if (max_factor < std::fabs(q_dot_ik(i) / limiter_params_.limits_vel[i]))
        {
            max_factor = std::fabs(q_dot_ik(i) / limiter_params_.limits_vel[i]);
            joint_index = i;
        }
    }

    if (max_factor > 1.0)
    {
        ROS_WARN_STREAM_THROTTLE(1, "Velocity limit surpassed (by Joint " << joint_index << "): Scaling ALL VELOCITIES with factor = " << max_factor);
        for (unsigned int i = 0; i < q_dot_ik.rows(); i++)
        {
            q_dot_norm(i) = q_dot_ik(i) / max_factor;
        }
    }

    return q_dot_norm;
}
/* END LimiterAllJointVelocities ********************************************************************************/

/* BEGIN LimiterAllJointAccelerations ******************************************************************************/
/**
 * Enforce limits on all joint velocities based on acceleration limits to keep direction.
 * Limits all velocities according to the limits_acc vector if necessary.
 */
KDL::JntArray LimiterAllJointAccelerations::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    KDL::JntArray q_dot_norm(q_dot_ik);

    ROS_WARN("LimiterAllJointAccelerations not yet implemented");

    return q_dot_norm;
}
/* END LimiterAllJointAccelerations *****************************************************************************/

/* BEGIN LimiterIndividualJointPositions ************************************************************************/
/**
 * This implementation calculates limits for the joint positions without keeping the direction.
 * Then for each corresponding joint velocity an individual factor for scaling is calculated and then used.
 */
KDL::JntArray LimiterIndividualJointPositions::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    KDL::JntArray q_dot_norm(q_dot_ik);
    double tolerance = limiter_params_.limits_tolerance / 180.0 * M_PI;

    for (unsigned int i = 0; i < q_dot_ik.rows(); i++)
    {
        if((limiter_params_.limits_max[i] - LIMIT_SAFETY_THRESHOLD <= q(i) && q_dot_ik(i) > 0) ||
           (limiter_params_.limits_min[i] + LIMIT_SAFETY_THRESHOLD >= q(i) && q_dot_ik(i) < 0))
        {
            ROS_ERROR_STREAM("Joint " << i << " violates its limits. Setting to Zero!");
            q_dot_norm(i) = 0.0;
        }

        double factor = 1.0;
        if (fabs(limiter_params_.limits_max[i] - q(i)) <= tolerance)  // Joint is close to the MAXIMUM limit
        {
            if (q_dot_ik(i) > 0.0)  // Joint moves towards the MAX limit
            {
                double temp = 1.0 / pow((0.5 + 0.5 * cos(M_PI * (q(i) + tolerance - limiter_params_.limits_max[i]) / tolerance)), 5.0);
                // double temp = tolerance / fabs(limiter_params_.limits_max[i] - q(i));
                factor = (temp > factor) ? temp : factor;
            }
        }

        if (fabs(q(i) - limiter_params_.limits_min[i]) <= tolerance)  // Joint is close to the MINIMUM limit
        {
            if (q_dot_ik(i) < 0.0)  // Joint moves towards the MIN limit
            {
                double temp = 1.0 / pow(0.5 + 0.5 * cos(M_PI * (q(i) - tolerance - limiter_params_.limits_min[i]) / tolerance), 5.0);
                // double temp = tolerance / fabs(q(i) - limiter_params_.limits_min[i]);
                factor = (temp > factor) ? temp : factor;
            }
        }
        q_dot_norm(i) = q_dot_norm(i) / factor;
    }

    return q_dot_norm;
}
/* END LimiterIndividualJointPositions **************************************************************************/

/* BEGIN LimiterIndividualJointVelocities ***********************************************************************/
/**
 * This implementation calculates limits for the joint velocities without keeping the direction.
 * For each joint velocity in the vector an individual factor for scaling is calculated and used.
 */
KDL::JntArray LimiterIndividualJointVelocities::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    KDL::JntArray q_dot_norm(q_dot_ik);

    for (unsigned int i = 0; i < q_dot_ik.rows(); i++)
    {
        double factor = 1.0;
        if (factor < std::fabs(q_dot_ik(i) / limiter_params_.limits_vel[i]))
        {
            factor = std::fabs(q_dot_ik(i) / limiter_params_.limits_vel[i]);
            q_dot_norm(i) = q_dot_ik(i) / factor;
        }
    }

    return q_dot_norm;
}
/* END LimiterIndividualJointVelocities *************************************************************************/

/* BEGIN LimiterIndividualJointAccelerations ********************************************************************/
/**
 * This implementation scales velocities based on given limits for joint accelerations without keeping the direction.
 * For each joint velocity in the vector an individual factor for scaling is calculated and used.
 */
KDL::JntArray LimiterIndividualJointAccelerations::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    KDL::JntArray q_dot_norm(q_dot_ik);

    ROS_WARN("LimiterIndividualJointAccelerations not yet implemented");

    return q_dot_norm;
}
/* END LimiterIndividualJointAccelerations **********************************************************************/
