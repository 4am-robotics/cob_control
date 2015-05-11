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
 *   This header contains the interface description of constraints
 *
 ****************************************************************/

#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

#include "cob_twist_controller/augmented_solver_data_types.h"
#include "cob_twist_controller/constraints/constraint_base.h"

/* BEGIN ConstraintParamsCA *************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsCA : public ConstraintParamsBase
{
    public:

        ConstraintParamsCA(const AugmentedSolverParams& params) : ConstraintParamsBase(params)
        {}

        virtual ~ConstraintParamsCA()
        {}

};
/* END ConstraintParamsCA ***************************************************************************************/

/* BEGIN CollisionAvoidance *************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename PRIO = uint32_t>
class CollisionAvoidance : public ConstraintBase<PRIO>
{
    public:

        CollisionAvoidance(const AugmentedSolverParams& params, PRIO prio) : ConstraintBase<PRIO>(params, prio)
        {}

        virtual void setConstraintParams(const ConstraintParamsBase* constraintParams);

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getSafeRegion() const;
        virtual double getPartialValue() const;


        virtual ~CollisionAvoidance()
        {
        }
};
/* END CollisionAvoidance ***************************************************************************************/

/* BEGIN ConstraintParamsJLA ************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsJLA : public ConstraintParamsBase
{
    public:

        ConstraintParamsJLA(const AugmentedSolverParams& params) : ConstraintParamsBase(params)
        {}

        virtual ~ConstraintParamsJLA()
        {}

};
/* END ConstraintParamsJLA **************************************************************************************/

/* BEGIN JointLimitAvoidance ************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename PRIO = uint32_t>
class JointLimitAvoidance : public ConstraintBase<PRIO>
{
    public:

        JointLimitAvoidance(const AugmentedSolverParams& params, PRIO prio) : ConstraintBase<PRIO>(params, prio)
        {}

        virtual void setConstraintParams(const ConstraintParamsBase* constraintParams);

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getSafeRegion() const;
        virtual double getPartialValue() const;


        virtual ~JointLimitAvoidance()
        {
        }
};
/* END JointLimitAvoidance **************************************************************************************/



#include "cob_twist_controller/constraints/constraint_impl.h" // implementation of templated class

#endif /* CONSTRAINT_H_ */
