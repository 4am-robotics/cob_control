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
#include "cob_twist_controller/callback_data_mediator.h"
#include "cob_twist_controller/task_stack/task_stack_controller.h"

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

        inline PRIO getPriority() const
        {
            return priority_;
        }

        virtual double getCriticalValue() const = 0;
        virtual double getActivationGain() const = 0;
        virtual void calculate() = 0;
        virtual double getValue() const = 0;
        virtual double getDerivativeValue() const = 0;
        virtual double getActivationThreshold() const = 0;
        virtual Eigen::VectorXd getPartialValues() const = 0;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution,
                                              const Eigen::MatrixXd& homogeneous_solution) const = 0;
        virtual void update(const JointStates& joint_states, const Eigen::MatrixXd& joint_pos_prediction, const Matrix6Xd_t& jacobian_data) = 0;
        virtual std::string getTaskId() const = 0;
        virtual ConstraintState getState() const = 0;
        virtual ConstraintTypes getType() const = 0;

        virtual Eigen::MatrixXd getTaskJacobian() const = 0;
        virtual Eigen::VectorXd getTaskDerivatives() const = 0;

        virtual Task_t createTask() = 0;

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
                       T_PARAMS params,
                       CallbackDataMediator& cbdm)
        : PriorityBase<PRIO>(prio),
          constraint_params_(params),
          callback_data_mediator_(cbdm),
          value_(0.0),
          derivative_value_(0.0),
          last_value_(0.0),
          last_time_(-0.1)
        {
            this->member_inst_cnt_ = instance_ctr_++;
        }

        virtual ~ConstraintBase()
        {}

        virtual void calculate() = 0;
        virtual double getActivationGain() const = 0;
        virtual std::string getTaskId() const = 0;
        virtual ConstraintTypes getType() const = 0;

        virtual double getCriticalValue() const
        {
            return 0.0;
        }

        virtual double getValue() const
        {
            return this->value_;
        }

        virtual double getDerivativeValue() const
        {
            return this->derivative_value_;
        }

        virtual Eigen::VectorXd getPartialValues() const
        {
            return this->partial_values_;
        }

        virtual void update(const JointStates& joint_states, const Eigen::MatrixXd& joint_pos_prediction, const Matrix6Xd_t& jacobian_data)
        {
            this->joint_states_ = joint_states;
            this->jacobian_data_ = jacobian_data;
            this->jnt_pos_prediction_ = joint_pos_prediction;
            this->callback_data_mediator_.fill(this->constraint_params_);

            this->calculate();
        }

        virtual ConstraintState getState() const
        {
            return this->state_;
        }

        virtual Eigen::MatrixXd getTaskJacobian() const
        {
            return Eigen::MatrixXd::Zero(1,1);
        }

        virtual Eigen::VectorXd getTaskDerivatives() const
        {
            return Eigen::VectorXd::Zero(1,1);
        }

        virtual Task_t createTask()
        {
            Task_t task(this->getPriority(),
                        this->getTaskId(),
                        this->getTaskJacobian(),
                        this->getTaskDerivatives(),
                        this->getType());
            return task;
        }

    protected:
        ConstraintState state_;
        JointStates joint_states_;
        Matrix6Xd_t jacobian_data_;
        T_PARAMS constraint_params_;
        CallbackDataMediator& callback_data_mediator_;
        Eigen::VectorXd partial_values_;
        Eigen::MatrixXd jnt_pos_prediction_;

        double derivative_value_;
        double value_;
        double last_value_;
        double last_time_;

        uint32_t member_inst_cnt_;
        static uint32_t instance_ctr_;
};

template <typename T_PARAMS, typename PRIO>
uint32_t ConstraintBase<T_PARAMS, PRIO>::instance_ctr_ = 0;

typedef boost::shared_ptr<PriorityBase<uint32_t> > ConstraintBase_t;

#endif /* CONSTRAINT_BASE_H_ */
