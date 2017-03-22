/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2017 \n
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
 *   Bruno Brito, email: Bruno.Brito@fraunhofer.de
 *
 * \date Date of creation: April, 2015
 *
 * \brief
 *   This header contains the interface description of limiters
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_LIMITERS_LIMITER_BASE_H
#define COB_TWIST_CONTROLLER_LIMITERS_LIMITER_BASE_H

#include "cob_twist_controller/cob_twist_controller_data_types.h"

/// Base class for joint/output limiters, defining interface methods.
class LimiterJointBase
{
    public:
        explicit LimiterJointBase(const LimiterParams& limiter_params) : limiter_params_(limiter_params)
        {}

        virtual ~LimiterJointBase() {}

        /**
         * Pure virtual method to mark as interface method which has to be implemented in inherited classes.
         * The intention is to implement a method which enforces limits to the q_dot_out vector according to
         * the calculated joint velocities and / or joint positions.
         * @param q_dot_ik The calculated joint velocities vector which has to be checked for limits.
         * @param q The last known joint positions.
         * @return Scaled joint velocities vector.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const = 0;

    protected:
        const LimiterParams& limiter_params_;

};

/// Base class for cartesian/input limiters, defining interface methods.
class LimiterCartesianBase
{
    public:
        explicit LimiterCartesianBase(const LimiterParams& limiter_params) : limiter_params_(limiter_params)
        {}

        virtual ~LimiterCartesianBase() {}

        /**
         * Pure virtual method to mark as interface method which has to be implemented in inherited classes.
         * The intention is to implement a method which enforces limits to the Cartesian twist vector according to
         * the output of the Cartesian controller.
         * @param v_in are the generated Cartesian twist velocities.
         * @return Scaled Cartesian twist vector.
         */
        virtual KDL::Twist enforceLimits(const KDL::Twist& v_in) const = 0;

    protected:
        const LimiterParams& limiter_params_;

};

#endif  // COB_TWIST_CONTROLLER_LIMITERS_LIMITER_BASE_H
