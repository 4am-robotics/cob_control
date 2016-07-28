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
 * \date Date of creation: May, 2015
 *
 * \brief
 *   Implementation of several constraints
 *
 ****************************************************************/

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

        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(tc_params, limiter_params, data_mediator);
        uint32_t startPrio = tc_params.priority_jla;
        for (uint32_t i = 0; i < tc_params.joints.size(); ++i)
        {
            // TODO: take care PRIO could be of different type than UINT32
            params.joint_ = tc_params.joints[i];
            params.joint_idx_ = static_cast<int32_t>(i);
            // copy of params will be created; priority increased with each joint.
            boost::shared_ptr<Jla_t > jla(new Jla_t(startPrio++, params, data_mediator));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
        }
    }
    else if (JLA_MID_ON == tc_params.constraint_jla)
    {
        // same params as for normal JLA
        typedef JointLimitAvoidanceMid<ConstraintParamsJLA, PRIO> JlaMid_t;
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(tc_params, limiter_params, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<JlaMid_t > jla(new JlaMid_t(tc_params.priority_jla, params, data_mediator));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if (JLA_INEQ_ON == tc_params.constraint_jla)
    {
        typedef JointLimitAvoidanceIneq<ConstraintParamsJLA, PRIO> Jla_t;
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(tc_params, limiter_params, data_mediator);
        uint32_t startPrio = tc_params.priority_jla;
        for (uint32_t i = 0; i < tc_params.joints.size(); ++i)
        {
            // TODO: take care PRIO could be of different type than UINT32
            params.joint_ = tc_params.joints[i];
            params.joint_idx_ = static_cast<int32_t>(i);
            // copy of params will be created; priority increased with each joint.
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
        uint32_t startPrio = tc_params.priority_ca;

        for (std::vector<std::string>::const_iterator it = tc_params.collision_check_links.begin();
             it != tc_params.collision_check_links.end(); it++)
        {
            ConstraintParamsCA params = ConstraintParamFactory<ConstraintParamsCA>::createConstraintParams(tc_params, limiter_params, data_mediator, *it);
            // TODO: take care PRIO could be of different type than UINT32
            boost::shared_ptr<CollisionAvoidance_t > ca(new CollisionAvoidance_t(startPrio--, params, data_mediator, jnt_to_jac, fk_solver_vel));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(ca));
        }
    }
    else
    {
        // CA_OFF selected.
        // Nothing to do here!
        // Create constraints will be called also in case of an unconstraint solver etc.
        // So the log would be filled unnecessarily.
    }

    return constraints;
}
/* END ConstraintsBuilder *******************************************************************************************/

// Collision Avoidance Constraint Implementation
#include "cob_twist_controller/constraints/constraint_ca_impl.h"

// Joint Limit Avoidance Constraint Implementation
#include "cob_twist_controller/constraints/constraint_jla_impl.h"

#endif  // COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_IMPL_H
