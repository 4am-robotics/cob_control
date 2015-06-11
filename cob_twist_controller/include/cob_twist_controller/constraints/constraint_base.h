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
#include "cob_twist_controller/constraints/self_motion_magnitude.h"
#include "cob_twist_controller/constraints/constraint_params.h"

template
<typename PRIO = uint32_t>
class PriorityBase
{
    public:

        PriorityBase(PRIO prio) : priority_(prio)
        {}

        virtual ~PriorityBase()
        {}

        inline void setPriority(PRIO prio)
        {
            this->priority_ = prio;
        }

        inline bool operator<(const PriorityBase& other) const
        {
            return ( this->priority_ < other.priority_ );
        }

        inline bool operator>(const PriorityBase& other) const
        {
            return ( this->priority_ > other.priority_ );
        }

        inline bool operator==(const PriorityBase& other) const
        {
            return ( this->priority_ == other.priority_ );
        }

        virtual double getValue() const = 0;
        virtual double getDerivativeValue() const = 0;
        virtual double getActivationThreshold() const = 0;
        virtual Eigen::VectorXd getPartialValues() const = 0;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution,
                                              const Eigen::MatrixXd& homogeneousSolution) const = 0;

    protected:
        PRIO priority_;
};


template
<typename T_PARAMS, typename PRIO = uint32_t> // if it is desired to implement an own priority class (ensure overriding of <, > and == parameters)
class ConstraintBase : public PriorityBase<PRIO>
{
    public:
        ConstraintBase(PRIO prio,
                       const KDL::JntArray& q,
                       T_PARAMS params)
        : PriorityBase<PRIO>(prio), jointPos_(q), constraintParams_(params)
        {}

        virtual ~ConstraintBase()
        {}

    protected:
        const KDL::JntArray& jointPos_;
        T_PARAMS constraintParams_;
};


typedef boost::shared_ptr<PriorityBase<uint32_t> > tConstraintBase;

#endif /* CONSTRAINT_BASE_H_ */
