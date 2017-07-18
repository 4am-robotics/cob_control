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


#ifndef COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_IMPL_H
#define COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_IMPL_H

#include <sstream>
#include <set>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <kdl/chainjnttojacsolver.hpp>

#include "cob_twist_controller/constraints/constraint.h"
#include "cob_twist_controller/constraints/constraint_params.h"

/* BEGIN ConstraintsBuilder *************************************************************************************/
/**
 * Static builder method to create constraints dependent on parameterization.
 */
template <typename PRIO>
std::set<ConstraintBase_t> ConstraintsBuilder<PRIO>::createConstraints(const TwistControllerParams& tc_params,
                                                                       const LimiterParams& limiter_params,
                                                                       KDL::ChainJntToJacSolver& jnt_to_jac,
                                                                       KDL::ChainFkSolverVel_recursive& fk_solver_vel,
                                                                       CallbackDataMediator& data_mediator)
{
    std::set<ConstraintBase_t> constraints;
    // Joint limit avoidance part
    if (JLA_ON == tc_params.constraint_jla)
    {
        typedef JointLimitAvoidance<ConstraintParamsJLA, PRIO> Jla_t;

        ConstraintParamsJLA params = ConstraintParamsJLA(tc_params.constraint_params.at(JLA), limiter_params);
        uint32_t startPrio = params.params_.priority;
        for (uint32_t i = 0; i < tc_params.joints.size(); ++i)
        {
            // TODO: take care PRIO could be of different type than UINT32
            params.joint_ = tc_params.joints[i];
            params.joint_idx_ = static_cast<int32_t>(i);
            boost::shared_ptr<Jla_t > jla(new Jla_t(startPrio++, params, data_mediator));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
        }
    }
    else if (JLA_MID_ON == tc_params.constraint_jla)
    {
        typedef JointLimitAvoidanceMid<ConstraintParamsJLA, PRIO> JlaMid_t;

        ConstraintParamsJLA params = ConstraintParamsJLA(tc_params.constraint_params.at(JLA), limiter_params);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<JlaMid_t > jla(new JlaMid_t(params.params_.priority, params, data_mediator));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if (JLA_INEQ_ON == tc_params.constraint_jla)
    {
        typedef JointLimitAvoidanceIneq<ConstraintParamsJLA, PRIO> Jla_t;

        ConstraintParamsJLA params = ConstraintParamsJLA(tc_params.constraint_params.at(JLA), limiter_params);
        uint32_t startPrio = params.params_.priority;
        for (uint32_t i = 0; i < tc_params.joints.size(); ++i)
        {
            // TODO: take care PRIO could be of different type than UINT32
            params.joint_ = tc_params.joints[i];
            params.joint_idx_ = static_cast<int32_t>(i);
            boost::shared_ptr<Jla_t > jla(new Jla_t(startPrio++, params, data_mediator));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
        }
    }
    else
    {
        // JLA_OFF selected.
    }

    // Collision avoidance part
    if (CA_ON == tc_params.constraint_ca)
    {
        typedef CollisionAvoidance<ConstraintParamsCA, PRIO> CollisionAvoidance_t;
        uint32_t startPrio = tc_params.constraint_params.at(CA).priority;

        for (std::vector<std::string>::const_iterator it = tc_params.collision_check_links.begin();
             it != tc_params.collision_check_links.end(); it++)
        {
            ConstraintParamsCA params = ConstraintParamsCA(tc_params.constraint_params.at(CA),tc_params.frame_names, *it);
            data_mediator.fill(params);
            // TODO: take care PRIO could be of different type than UINT32
            boost::shared_ptr<CollisionAvoidance_t > ca(new CollisionAvoidance_t(startPrio--, params, data_mediator, jnt_to_jac, fk_solver_vel));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(ca));
        }
    }
    else
    {
        // CA_OFF selected.
    }

    return constraints;
}
/* END ConstraintsBuilder *******************************************************************************************/

// Collision Avoidance Constraint Implementation
#include "cob_twist_controller/constraints/constraint_ca_impl.h"

// Joint Limit Avoidance Constraint Implementation
#include "cob_twist_controller/constraints/constraint_jla_impl.h"

#endif  // COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_IMPL_H
