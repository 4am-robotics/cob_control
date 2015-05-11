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

#ifndef CONSTRAINT_BASE_H_
#define CONSTRAINT_BASE_H_

#include <stdint.h>

#include "cob_twist_controller/augmented_solver_data_types.h"

class ConstraintParamsBase
{
    public:

        ConstraintParamsBase(const AugmentedSolverParams& params) : params_(params)
        {}

        virtual ~ConstraintParamsBase()
        {}

    protected:
        const AugmentedSolverParams& params_;
};

template
<typename PRIO = uint32_t> // if it is desired to implement an own priority class (ensure overriding of <, > and == parameters)
class ConstraintBase
{
    public:

        ConstraintBase(const AugmentedSolverParams& params,
                       PRIO prio)
        : params_(params), priority_(prio)
        {}

        virtual ~ConstraintBase()
        {}

        inline void setPriority(PRIO prio)
        {
            this->priority_ = prio;
        }

        virtual void setConstraintParams(const ConstraintParamsBase* constraintParams) = 0;
        virtual double getValue() const = 0;
        virtual double getDerivativeValue() const = 0;
        virtual double getSafeRegion() const = 0;
        virtual double getPartialValue() const = 0;

        inline bool operator<(const ConstraintBase& other) const
        {
            return ( this->priority_ < other.priority_ );
        }

        inline bool operator>(const ConstraintBase& other) const
        {
            return ( this->priority_ > other.priority_ );
        }

        inline bool operator==(const ConstraintBase& other) const
        {
            return ( this->priority_ == other.priority_ );
        }

    protected:
        const AugmentedSolverParams& params_;
        PRIO priority_;
};

//class CompareConstraint
//{
//    public:
//        inline bool operator()(const ConstraintBase& c1, const ConstraintBase& c2) const
//        {
//            return (c1 > c2);
//        }
//};

#endif /* CONSTRAINT_BASE_H_ */
