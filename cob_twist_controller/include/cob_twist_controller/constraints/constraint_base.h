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

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraints/self_motion_magnitude.h"
#include "cob_twist_controller/constraints/constraint_params.h"

/**
 * Main base class for all derived constraints. Used to create abstract containers that can be filled with concrete constraints.
 * @tparam PRIO A priority class that has operators <, > and == for comparison overridden. Default uint comparison.
 */
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
        virtual Eigen::VectorXd getPartialValues() = 0;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution,
                                              const Eigen::MatrixXd& homogeneous_solution) const = 0;
        virtual Eigen::MatrixXd getObstacleAvoidancePointJac() const = 0;
        virtual Eigen::Vector3d getDistanceVector() const = 0;

    protected:
        PRIO priority_;
};


/**
 * Base class for all derived constraints. Used to represent a common data structure for all concrete constraints.
 * @tparam T_PARAMS A specific constraint parameter class.
 * @tparam PRIO See base class.
 */
template
<typename T_PARAMS, typename PRIO = uint32_t>
class ConstraintBase : public PriorityBase<PRIO>
{
    public:

        /**
         * @param prio A priority value / object.
         * @param q The joint states.
         * @param params The parameters for the constraint to parameterize the calculation of the cost function values.
         */
        ConstraintBase(PRIO prio,
                       const JointStates& joint_states,
                       T_PARAMS params)
        : PriorityBase<PRIO>(prio), joint_states_(joint_states), constraint_params_(params)
        {}

        virtual ~ConstraintBase()
        {}

    protected:
        const JointStates& joint_states_;
        T_PARAMS constraint_params_;
};


typedef boost::shared_ptr<PriorityBase<uint32_t> > tConstraintBase;

#endif /* CONSTRAINT_BASE_H_ */
