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
 *   This header contains the class definitions of all limiter implementations.
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_LIMITERS_LIMITER_H
#define COB_TWIST_CONTROLLER_LIMITERS_LIMITER_H

#include <vector>

#include "cob_twist_controller/limiters/limiter_base.h"

#define LIMIT_SAFETY_THRESHOLD 0.1/180.0*M_PI

/* BEGIN LimiterJointContainer *******************************************************************************/
/// Joint Container for limiters, implementing interface methods.
class LimiterJointContainer : public LimiterJointBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterJointBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        /**
         * Initialization for the container.
         */
        void init();

        virtual ~LimiterJointContainer();

        explicit LimiterJointContainer(const LimiterParams& limiter_params)
            : LimiterJointBase(limiter_params)
        {}

    protected:
        std::vector<const LimiterJointBase*> output_limiters_;
        typedef std::vector<const LimiterJointBase*>::const_iterator output_LimIter_t;

        /**
         * Add method
         * @param lb An implementation of a limiter.
         */
        void add(const LimiterJointBase* lb);

        /**
         * Erase all
         */
        void eraseAll();
};
/* END LimiterJointContainer *****************************************************************************************/

/* BEGIN LimiterContainer *******************************************************************************/
/// Container for limiters, implementing interface methods.
class LimiterCartesianContainer : public LimiterCartesianBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterCartesianBase for more details on params and returns.
         */

        virtual KDL::Twist enforceLimits(const KDL::Twist& v_in) const;

        /**
         * Initialization for the container.
         */
        void init();

        virtual ~LimiterCartesianContainer();

        explicit LimiterCartesianContainer(const LimiterParams& limiter_params)
            : LimiterCartesianBase(limiter_params)
        {}

    protected:
        std::vector<const LimiterCartesianBase*> input_limiters_;
        typedef std::vector<const LimiterCartesianBase*>::const_iterator input_LimIter_t;

        /**
         * Add method
         * @param lb An implementation of a limiter.
         */
        void add(const LimiterCartesianBase* lb);

        /**
         * Erase all
         */
        void eraseAll();
};
/* END LimiterCartesianContainer *****************************************************************************************/

/* BEGIN LimiterAllJointPositions *******************************************************************************/
/// Class for limiters, declaring the method to limit all joint positions.
class LimiterAllJointPositions : public LimiterJointBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterJointBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        explicit LimiterAllJointPositions(const LimiterParams& limiter_params) :
            LimiterJointBase(limiter_params)
        {}
};
/* END LimiterAllJointPositions **********************************************************************************/

/* BEGIN LimiterAllJointVelocities *******************************************************************************/
/// Class for joint velocity limiter (all scaled to keep direction), implementing interface methods.
class LimiterAllJointVelocities : public LimiterJointBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterJointBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        explicit LimiterAllJointVelocities(const LimiterParams& limiter_params) :
            LimiterJointBase(limiter_params)
        {}
};
/* END LimiterAllJointVelocities *********************************************************************************/

/* BEGIN LimiterAllJointAccelerations ****************************************************************************/
/// Class for joint acceleration limiter (all scaled to keep direction), implementing interface methods.
class LimiterAllJointAccelerations : public LimiterJointBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterJointBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        explicit LimiterAllJointAccelerations(const LimiterParams& limiter_params) :
            LimiterJointBase(limiter_params)
        {}
};
/* END LimiterAllJointAccelerations ******************************************************************************/

/* BEGIN LimiterIndividualJointPositions *************************************************************************/
/// Class for a limiter, declaring a method to limit joint positions individually
class LimiterIndividualJointPositions : public LimiterJointBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterJointBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        explicit LimiterIndividualJointPositions(const LimiterParams& limiter_params) :
            LimiterJointBase(limiter_params)
        {}
};
/* END LimiterIndividualJointPositions **************************************************************************/

/* BEGIN LimiterIndividualJointVelocities ***********************************************************************/
/// Class for joint velocity limiter (individually scaled -> changes direction), implementing interface methods.
class LimiterIndividualJointVelocities : public LimiterJointBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterJointBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        explicit LimiterIndividualJointVelocities(const LimiterParams& limiter_params) :
            LimiterJointBase(limiter_params)
        {}
};
/* END LimiterIndividualJointVelocities *************************************************************************/

/* BEGIN LimiterIndividualJointAccelerations ***********************************************************************/
/// Class for joint acceleration limiter (individually scaled -> changes direction), implementing interface methods.
class LimiterIndividualJointAccelerations : public LimiterJointBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterJointBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        explicit LimiterIndividualJointAccelerations(const LimiterParams& limiter_params) :
            LimiterJointBase(limiter_params)
        {}
};
/* END LimiterIndividualJointAccelerations *************************************************************************/

/* BEGIN LimiterCartesianVelocities ***********************************************************************/
/// Class for limiting the cartesian velocities commands in order to guarantee a BIBO system.
class LimiterCartesianVelocities : public LimiterCartesianBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterCartesianBase for more details on params and returns.
         */

        virtual KDL::Twist enforceLimits(const KDL::Twist& v_in) const;

        explicit LimiterCartesianVelocities(const LimiterParams& limiter_params) :
            LimiterCartesianBase(limiter_params)
        {}
};
/* END LimiterCartesianVelocities *************************************************************************/

#endif  // COB_TWIST_CONTROLLER_LIMITERS_LIMITER_H
