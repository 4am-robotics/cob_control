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
#ifndef LIMITER_BASE_H_
#define LIMITER_BASE_H_

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

/// Base class for limiters, defining interface methods.
class LimiterBase
{
    public:
        /**
         * Pure virtual method to mark as interface method which has to be implemented in inherited classes.
         * The intention is to implement a method which enforces limits to the q_dot_out vector according to
         * the calculated joint velocities and / or joint positions.
         * @param q_dot_ik The calculated joint velocities vector which has to be checked for limits.
         * @param q The last known joint positions.
         * @return Scaled joint velocities vector.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const = 0;

        virtual ~LimiterBase() = 0;

    protected:
        LimiterBase(const TwistControllerParams &tcParams, const KDL::Chain &chain);

        const TwistControllerParams &tcParams_;
        const KDL::Chain &chain_;
};

#endif /* LIMITER_BASE_H_ */
