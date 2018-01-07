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
