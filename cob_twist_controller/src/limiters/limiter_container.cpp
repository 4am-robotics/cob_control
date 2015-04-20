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
#include <vector>

#include <ros/ros.h>
#include <kdl/chain.hpp>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/limiters/limiter_container.h"
#include "cob_twist_controller/limiters/limiter_all_joint_positions.h"
#include "cob_twist_controller/limiters/limiter_all_joint_velocities.h"
#include "cob_twist_controller/limiters/limiter_individual_joint_positions.h"
#include "cob_twist_controller/limiters/limiter_individual_joint_velocities.h"

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

    if(this->tcParams_.enforce_limits)
    {
        if(this->tcParams_.keep_direction)
        {
            this->add(new LimiterAllJointPositions(this->tcParams_, this->chain_));
            this->add(new LimiterAllJointVelocities(this->tcParams_, this->chain_));
        }
        else
        {
            this->add(new LimiterIndividualJointPositions(this->tcParams_, this->chain_));
            this->add(new LimiterIndividualJointVelocities(this->tcParams_, this->chain_));
        }
    }
    else
    {
        // nothing to do yet
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
