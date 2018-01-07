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


#ifndef COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_BASE_H
#define COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_BASE_H

#include <set>
#include <string>
#include <limits>
#include <stdint.h>
#include <boost/shared_ptr.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <Eigen/Core>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraints/constraint_params.h"
#include "cob_twist_controller/callback_data_mediator.h"
#include "cob_twist_controller/task_stack/task_stack_controller.h"
/**
 * Main base class for all derived constraints. Used to create abstract containers that can be filled with concrete constraints.
 * @tparam PRIO A priority class that has operators <, > and == for comparison overridden. To return the priority as a computable double it must override "operator double() const"! Default uint comparison.
 */
template
<typename PRIO = uint32_t>
class PriorityBase
{
    public:
        explicit PriorityBase(PRIO prio): priority_(prio)
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

        /// Ensure priority class has overwritten double() operator.
        inline double getPriorityAsNum() const
        {
            return static_cast<double>(priority_);
        }

        virtual Task_t createTask() = 0;
        virtual std::string getTaskId() const = 0;
        virtual ConstraintState getState() const = 0;
        virtual Eigen::MatrixXd getTaskJacobian() const = 0;
        virtual Eigen::VectorXd getTaskDerivatives() const = 0;

        virtual void update(const JointStates& joint_states, const KDL::JntArrayVel& joints_prediction, const Matrix6Xd_t& jacobian_data) = 0;
        virtual void calculate() = 0;
        virtual double getValue() const = 0;
        virtual double getDerivativeValue() const = 0;
        virtual Eigen::VectorXd getPartialValues() const = 0;
        virtual double getPredictionValue() const = 0;

        virtual double getActivationGain() const = 0;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution,
                                              const Eigen::MatrixXd& homogeneous_solution) const = 0;

    protected:
        PRIO priority_;

        virtual double getCriticalValue() const = 0;
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
          prediction_value_(std::numeric_limits<double>::max()),
          last_value_(0.0),
          last_time_(ros::Time::now()),
          last_pred_time_(ros::Time::now())
        {
            this->member_inst_cnt_ = instance_ctr_++;
        }

        virtual ~ConstraintBase()
        {}

        virtual Task_t createTask()
        {
            Task_t task(this->getPriority(),
                        this->getTaskId(),
                        this->getTaskJacobian(),
                        this->getTaskDerivatives());
            return task;
        }

        virtual std::string getTaskId() const = 0;

        virtual ConstraintState getState() const
        {
            return this->state_;
        }

        virtual Eigen::MatrixXd getTaskJacobian() const
        {
            return Eigen::MatrixXd::Zero(1, 1);
        }

        virtual Eigen::VectorXd getTaskDerivatives() const
        {
            return Eigen::VectorXd::Zero(1, 1);
        }

        virtual void update(const JointStates& joint_states, const KDL::JntArrayVel& joints_prediction, const Matrix6Xd_t& jacobian_data)
        {
            // ROS_INFO_STREAM("ConstraintBase::update: joint_states.current_q_.rows: " << joint_states.current_q_.rows());
            // ROS_INFO_STREAM("ConstraintBase::update: joints_prediction.q.rows: " << joints_prediction.q.rows());
            // ROS_INFO_STREAM("ConstraintBase::update: jacobian_data.cols: " << jacobian_data.cols());

            this->joint_states_ = joint_states;
            this->jacobian_data_ = jacobian_data;
            this->jnts_prediction_ = joints_prediction;
            this->callback_data_mediator_.fill(this->constraint_params_);
            this->calculate();
        }

        virtual void calculate() = 0;

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

        virtual double getPredictionValue() const
        {
            return this->prediction_value_;
        }

        virtual double getActivationGain() const = 0;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution,
                                              const Eigen::MatrixXd& homogeneous_solution) const = 0;

    protected:
        ConstraintState state_;
        T_PARAMS constraint_params_;
        CallbackDataMediator& callback_data_mediator_;

        JointStates joint_states_;
        KDL::JntArrayVel jnts_prediction_;
        Matrix6Xd_t jacobian_data_;

        double value_;
        double derivative_value_;
        Eigen::VectorXd partial_values_;
        double prediction_value_;
        double last_value_;
        ros::Time last_time_;
        ros::Time last_pred_time_;

        uint32_t member_inst_cnt_;
        static uint32_t instance_ctr_;

        virtual double getCriticalValue() const
        {
            return 0.0;
        }
};

template <typename T_PARAMS, typename PRIO>
uint32_t ConstraintBase<T_PARAMS, PRIO>::instance_ctr_ = 0;

typedef boost::shared_ptr<PriorityBase<uint32_t> > ConstraintBase_t;

#endif  // COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_BASE_H
