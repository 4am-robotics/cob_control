/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef COB_TWIST_CONTROLLER_LIMITERS_LIMITER_H
#define COB_TWIST_CONTROLLER_LIMITERS_LIMITER_H

#include <vector>

#include "cob_twist_controller/limiters/limiter_base.h"

#define LIMIT_SAFETY_THRESHOLD 0.1/180.0*M_PI

/* BEGIN LimiterJointContainer *******************************************************************************/
/// Container for limiters, implementing interface methods.
class LimiterContainer
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterJointBase for more details on params and returns.
         */
        virtual KDL::Twist enforceLimits(const KDL::Twist& v_in) const;
        virtual KDL::JntArray enforceLimits(const KDL::JntArray& q_dot_ik, const KDL::JntArray& q) const;

        /**
         * Initialization for the container.
         */
        void init();

        virtual ~LimiterContainer();

        explicit LimiterContainer(const LimiterParams& limiter_params) :
            limiter_params_(limiter_params)
        {}

    protected:
        const LimiterParams& limiter_params_;

        std::vector<const LimiterCartesianBase*> input_limiters_;
        std::vector<const LimiterJointBase*> output_limiters_;
        typedef std::vector<const LimiterCartesianBase*>::const_iterator input_LimIter_t;
        typedef std::vector<const LimiterJointBase*>::const_iterator output_LimIter_t;

        /**
         * Add method
         * @param lb An implementation of a limiter.
         */
        void add(const LimiterCartesianBase* lb);
        void add(const LimiterJointBase* lb);

        /**
         * Erase all
         */
        void eraseAll();
};
/* END LimiterJointContainer *****************************************************************************************/

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

/* BEGIN LimiterAllCartesianVelocities ***********************************************************************/
/// Class for limiting the cartesian velocities commands in order to guarantee a BIBO system (all scaled to keep direction).
class LimiterAllCartesianVelocities : public LimiterCartesianBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterCartesianBase for more details on params and returns.
         */
        virtual KDL::Twist enforceLimits(const KDL::Twist& v_in) const;

        explicit LimiterAllCartesianVelocities(const LimiterParams& limiter_params) :
            LimiterCartesianBase(limiter_params)
        {}
};
/* END LimiterAllCartesianVelocities *************************************************************************/

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

/* BEGIN LimiterIndividualCartesianVelocities ***********************************************************************/
/// Class for limiting the cartesian velocities commands in order to guarantee a BIBO system (individually scaled -> changes direction).
class LimiterIndividualCartesianVelocities : public LimiterCartesianBase
{
    public:
        /**
         * Specific implementation of enforceLimits-method.
         * See base class LimiterCartesianBase for more details on params and returns.
         */
        virtual KDL::Twist enforceLimits(const KDL::Twist& v_in) const;

        explicit LimiterIndividualCartesianVelocities(const LimiterParams& limiter_params) :
            LimiterCartesianBase(limiter_params)
        {}
};
/* END LimiterIndividualCartesianVelocities *************************************************************************/

#endif  // COB_TWIST_CONTROLLER_LIMITERS_LIMITER_H
