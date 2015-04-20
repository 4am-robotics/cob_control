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
 *   This module contains the descrption of a method to limit individual joint positions.
 *
 ****************************************************************/
#ifndef LIMITER_INDIVIDUAL_JOINT_POSITIONS_H_
#define LIMITER_INDIVIDUAL_JOINT_POSITIONS_H_

#include "cob_twist_controller/limiters/limiter_base.h"

/// Class for a limiter, declaring a method to limit joint positions individually
class LimiterIndividualJointPositions : public LimiterBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        LimiterIndividualJointPositions(const TwistControllerParams &tcParams, const KDL::Chain &chain) :
            LimiterBase(tcParams, chain)
        {
        }
};

#endif /* LIMITER_INDIVIDUAL_JOINT_POSITIONS_H_ */
