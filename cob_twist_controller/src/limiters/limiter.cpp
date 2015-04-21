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
 *   methods to limit joint positions / velocities.
 *
 ****************************************************************/
#include <ros/ros.h>
#include <kdl/chain.hpp>

#include "cob_twist_controller/limiters/limiter.h"

/* BEGIN LimiterContainer ***************************************************************************************/
/**
 * This implementation calls enforce limits on all registered Limiters in the limiters vector.
 * The method is based on the last calculation of q_dot.
 */
KDL::JntArray LimiterContainer::enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const
{
    // If nothing to do just return q_dot.
    KDL::JntArray tmp_q_dots(q_dot_ik);
    for (limIter it = this->limiters.begin(); it != this->limiters.end(); it++)
    {
        tmp_q_dots = (*it)->enforceLimits(tmp_q_dots, q);
    }

    return tmp_q_dots;
}

/**
 * Building the limiters vector according the the chosen parameters.
 */
void LimiterContainer::init()
{
    this->eraseAll();

    if(this->tcParams_.keep_direction)
    {
        if(this->tcParams_.enforce_pos_limits)
        {
            this->add(new LimiterAllJointPositions(this->tcParams_, this->chain_));
        }

        if(this->tcParams_.enforce_vel_limits)
        {
            this->add(new LimiterAllJointVelocities(this->tcParams_, this->chain_));
        }
    }
    else
    {
        if(this->tcParams_.enforce_pos_limits)
        {
            this->add(new LimiterIndividualJointPositions(this->tcParams_, this->chain_));
        }

        if(this->tcParams_.enforce_vel_limits)
        {
            this->add(new LimiterIndividualJointVelocities(this->tcParams_, this->chain_));
        }
    }
}

/**
 * Deletes all limiters and clears the vector holding them.
 */
void LimiterContainer::eraseAll()
{
    for (uint32_t cnt = 0; cnt < this->limiters.capacity(); ++cnt)
    {
        const LimiterBase* lb = this->limiters[cnt];
        delete(lb);
    }

    this->limiters.clear();
}

/**
 * Adding new limiters to the vector.
 */
void LimiterContainer::add(const LimiterBase *lb)
{
    this->limiters.push_back(lb);
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
/* END LimiterAllJointPositions *********************************************************************************/

/* BEGIN LimiterAllJointVelocities ******************************************************************************/
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
/* END LimiterAllJointVelocities ********************************************************************************/

/* BEGIN LimiterIndividualJointPositions ************************************************************************/
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
/* END LimiterIndividualJointPositions **************************************************************************/

/* BEGIN LimiterIndividualJointVelocities ***********************************************************************/
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
/* END LimiterIndividualJointVelocities *************************************************************************/
