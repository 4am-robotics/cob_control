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
 *   This header contains the description of a method to limit joint velocities individually
 *
 ****************************************************************/
#ifndef LIMITER_INDIVIDUAL_JOINT_VELOCITIES_H_
#define LIMITER_INDIVIDUAL_JOINT_VELOCITIES_H_

#include "cob_twist_controller/limiters/limiter_base.h"

/// Class for joint velocity limiter (individually scaled -> changes direction), implementing interface methods.
class LimiterIndividualJointVelocities : public LimiterBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        LimiterIndividualJointVelocities(const TwistControllerParams &tcParams, const KDL::Chain &chain) :
            LimiterBase(tcParams, chain)
        {
        }
};

#endif /* LIMITER_INDIVIDUAL_JOINT_VELOCITIES_H_ */
