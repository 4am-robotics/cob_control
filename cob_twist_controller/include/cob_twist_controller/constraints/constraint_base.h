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

#include <set>
#include <stdint.h>
#include <boost/shared_ptr.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Core>

#include "cob_twist_controller/augmented_solver_data_types.h"

class ConstraintParamsBase
{
    public:

        ConstraintParamsBase(const AugmentedSolverParams& params) : params_(params)
        {}

        virtual ~ConstraintParamsBase()
        {}

        virtual const AugmentedSolverParams& getAugmentedSolverParams() const
        {
            return this->params_;
        }

    protected:
        const AugmentedSolverParams& params_;
};

template
<typename PRIO = uint32_t> // if it is desired to implement an own priority class (ensure overriding of <, > and == parameters)
class ConstraintBase
{
    public:

        ConstraintBase(PRIO prio, const KDL::JntArray& q)
        : priority_(prio), jointPos_(q)
        {
            this->constraintParams_ = NULL;
        }

        virtual ~ConstraintBase()
        {}

        inline void setPriority(PRIO prio)
        {
            this->priority_ = prio;
        }

        virtual void setConstraintParams(const ConstraintParamsBase* constraintParams)
        {
            this->constraintParams_ = constraintParams;
        }


        virtual double getValue() const = 0;
        virtual double getDerivativeValue() const = 0;
        virtual double getSafeRegion() const = 0;
        virtual Eigen::VectorXd getPartialValues() const = 0;
        virtual double getStepSize() const
        {
            return 0.0;
        }


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
        PRIO priority_;
        const KDL::JntArray& jointPos_;
        const ConstraintParamsBase* constraintParams_;
};

typedef boost::shared_ptr<ConstraintBase<> > tConstraintBase;

#endif /* CONSTRAINT_BASE_H_ */
