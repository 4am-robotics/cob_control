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
#ifndef LIMITER_ALL_JOINT_POSITIONS_H_
#define LIMITER_ALL_JOINT_POSITIONS_H_

#include "cob_twist_controller/limiters/limiter_base.h"

/// Class for limiters, declaring the method to limit all joint positions.
class LimiterAllJointPositions : public LimiterBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        LimiterAllJointPositions(const TwistControllerParams &tcParams, const KDL::Chain &chain) :
            LimiterBase(tcParams, chain)
        {
        }
};

#endif /* LIMITER_ALL_JOINT_POSITIONS_H_ */
