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
