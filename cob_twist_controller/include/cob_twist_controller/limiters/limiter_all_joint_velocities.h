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
#ifndef LIMITER_ALL_JOINT_VELOCITIES_H_
#define LIMITER_ALL_JOINT_VELOCITIES_H_

#include "cob_twist_controller/limiters/limiter_base.h"

/// Class for joint velocity limiter (all scaled to keep direction), implementing interface methods.
class LimiterAllJointVelocities : public LimiterBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        LimiterAllJointVelocities(const TwistControllerParams &tcParams, const KDL::Chain &chain) :
            LimiterBase(tcParams, chain)
        {
        }
};

#endif /* LIMITER_ALL_JOINT_VELOCITIES_H_ */
