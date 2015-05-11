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

#ifndef CONSTRAINT_IMPL_H_
#define CONSTRAINT_IMPL_H_

#include "cob_twist_controller/constraints/constraint.h"
#include "ros/ros.h"

/* BEGIN ConstraintsBuilder *************************************************************************************/
/**
 * Static builder method to create damping methods dependent on parameterization.
 */
template <typename PRIO>
std::set<tConstraintBase> ConstraintsBuilder<PRIO>::create_constraints(AugmentedSolverParams &augmentedSolverParams)
{
    std::set<tConstraintBase> constraints;

    tConstraintBase ca(new CollisionAvoidance<PRIO>(100)); // TODO: take case PRIO could be of different type than UINT32
    ca->setConstraintParams(new ConstraintParamsCA(augmentedSolverParams));


    constraints.insert(ca);

//    ConstraintBase<PRIO> *db = NULL;
//    switch(augmentedSolverParams.damping_method)
//    {
//        case NONE:
//            db = new DampingNone(augmentedSolverParams);
//            break;
//        case CONSTANT:
//            db = new DampingConstant(augmentedSolverParams);
//            break;
//        case MANIPULABILITY:
//            db = new DampingManipulability(augmentedSolverParams);
//            break;
//        default:
//            ROS_ERROR("DampingMethod %d not defined! Aborting!", augmentedSolverParams.damping_method);
//            break;
//    }

    return constraints;
}
/* END ConstraintsBuilder *******************************************************************************************/

/* BEGIN CollisionAvoidance *************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename PRIO>
void CollisionAvoidance<PRIO>::setConstraintParams(const ConstraintParamsBase* constraintParams)
{
    const ConstraintParamsCA* cpca = dynamic_cast<const ConstraintParamsCA*>(constraintParams);
}

template <typename PRIO>
double CollisionAvoidance<PRIO>::getValue() const
{
    return 0.0;
}

template <typename PRIO>
double CollisionAvoidance<PRIO>::getDerivativeValue() const
{
    return 0.0;
}

template <typename PRIO>
double CollisionAvoidance<PRIO>::getSafeRegion() const
{
    return 0.0;
}

template <typename PRIO>
double CollisionAvoidance<PRIO>::getPartialValue() const
{
    return 0.0;
}
/* END CollisionAvoidance ***************************************************************************************/

/* BEGIN JointLimitAvoidance ************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename PRIO>
void JointLimitAvoidance<PRIO>::setConstraintParams(const ConstraintParamsBase* constraintParams)
{
    const ConstraintParamsJLA* cpca = dynamic_cast<const ConstraintParamsJLA*>(constraintParams);
}

template <typename PRIO>
double JointLimitAvoidance<PRIO>::getValue() const
{
    return 0.0;
}

template <typename PRIO>
double JointLimitAvoidance<PRIO>::getDerivativeValue() const
{
    return 0.0;
}

template <typename PRIO>
double JointLimitAvoidance<PRIO>::getSafeRegion() const
{
    return 0.0;
}

template <typename PRIO>
double JointLimitAvoidance<PRIO>::getPartialValue() const
{
    return 0.0;
}
/* END JointLimitAvoidance **************************************************************************************/

#endif /* CONSTRAINT_IMPL_H_ */
