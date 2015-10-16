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
 *   This header contains the class definitions of all limiter implementations.
 *
 ****************************************************************/
#ifndef LIMITER_H_
#define LIMITER_H_

#include "cob_twist_controller/limiters/limiter_base.h"
#include <vector>

/* BEGIN LimiterContainer *******************************************************************************/
/// Container for limiters, implementing interface methods.
class LimiterContainer : public LimiterBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        /**
         * Initialization for the container.
         */
        void init();

        virtual ~LimiterContainer();

        LimiterContainer(const TwistControllerParams& tc_params, const KDL::Chain& chain)
            : LimiterBase(tc_params, chain)
        {

        }

    protected:
        std::vector<const LimiterBase*> limiters_;
        typedef std::vector<const LimiterBase*>::const_iterator LimIter_t;

        /**
         * Add method
         * @param lb An implementation of a limiter.
         */
        void add(const LimiterBase* lb);

        /**
         * Erase all
         */
        void eraseAll();
};
/* END LimiterContainer *****************************************************************************************/

/* BEGIN LimiterAllJointPositions *******************************************************************************/
/// Class for limiters, declaring the method to limit all joint positions.
class LimiterAllJointPositions : public LimiterBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        LimiterAllJointPositions(const TwistControllerParams& tc_params, const KDL::Chain& chain) :
            LimiterBase(tc_params, chain)
        {
        }
};
/* END LimiterAllJointPositions **********************************************************************************/

/* BEGIN LimiterAllJointVelocities *******************************************************************************/
/// Class for joint velocity limiter (all scaled to keep direction), implementing interface methods.
class LimiterAllJointVelocities : public LimiterBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        LimiterAllJointVelocities(const TwistControllerParams& tc_params, const KDL::Chain& chain) :
            LimiterBase(tc_params, chain)
        {
        }
};
/* END LimiterAllJointVelocities *********************************************************************************/

/* BEGIN LimiterIndividualJointPositions *************************************************************************/
/// Class for a limiter, declaring a method to limit joint positions individually
class LimiterIndividualJointPositions : public LimiterBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        LimiterIndividualJointPositions(const TwistControllerParams& tc_params, const KDL::Chain& chain) :
            LimiterBase(tc_params, chain)
        {
        }
};
/* END LimiterIndividualJointPositions **************************************************************************/

/* BEGIN LimiterIndividualJointVelocities ***********************************************************************/
/// Class for joint velocity limiter (individually scaled -> changes direction), implementing interface methods.
class LimiterIndividualJointVelocities : public LimiterBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterBase for more details on params and returns.
         */
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        LimiterIndividualJointVelocities(const TwistControllerParams& tc_params, const KDL::Chain& chain) :
            LimiterBase(tc_params, chain)
        {
        }
};
/* END LimiterIndividualJointVelocities *************************************************************************/

#endif /* LIMITER_H_ */
