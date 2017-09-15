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


#ifndef COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_H
#define COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_H

#include <set>
#include <string>
#include <limits>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraints/constraint_base.h"
#include "cob_twist_controller/callback_data_mediator.h"

/* BEGIN ConstraintsBuilder *************************************************************************************/
/// Class providing a static method to create constraints.
template <typename PRIO = uint32_t>
class ConstraintsBuilder
{
    public:
        static std::set<ConstraintBase_t> createConstraints(const TwistControllerParams& params,
                                                            const LimiterParams& limiter_params,
                                                            KDL::ChainJntToJacSolver& jnt_to_jac_,
                                                            KDL::ChainFkSolverVel_recursive& fk_solver_vel,
                                                            CallbackDataMediator& data_mediator);

    private:
        ConstraintsBuilder() {}
        ~ConstraintsBuilder() {}
};
/* END ConstraintsBuilder ***************************************************************************************/

/* BEGIN CollisionAvoidance *************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename T_PARAMS, typename PRIO = uint32_t>
class CollisionAvoidance : public ConstraintBase<T_PARAMS, PRIO>
{
    public:
        CollisionAvoidance(PRIO prio,
                           T_PARAMS constraint_params,
                           CallbackDataMediator& cbdm,
                           KDL::ChainJntToJacSolver& jnt_to_jac,
                           KDL::ChainFkSolverVel_recursive& fk_solver_vel) :
            ConstraintBase<T_PARAMS, PRIO>(prio, constraint_params, cbdm),
            jnt_to_jac_(jnt_to_jac),
            fk_solver_vel_(fk_solver_vel)
        {}

        virtual ~CollisionAvoidance()
        {}

        virtual std::string getTaskId() const;
        virtual Eigen::MatrixXd getTaskJacobian() const;
        virtual Eigen::VectorXd getTaskDerivatives() const;

        virtual void calculate();

        virtual double getActivationGain() const;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution,
                                              const Eigen::MatrixXd& homogeneous_solution) const;

        double getActivationGain(double current_cost_func_value) const;
        double getSelfMotionMagnitude(double current_cost_func_value) const;

    private:
        virtual double getCriticalValue() const;

        void calcValue();
        void calcDerivativeValue();
        void calcPartialValues();
        void calcPredictionValue();
        double getActivationThresholdWithBuffer() const;

        KDL::ChainJntToJacSolver& jnt_to_jac_;
        KDL::ChainFkSolverVel_recursive& fk_solver_vel_;

        Eigen::VectorXd values_;
        Eigen::VectorXd derivative_values_;
        Eigen::MatrixXd task_jacobian_;
};
/* END CollisionAvoidance ***************************************************************************************/

/* BEGIN JointLimitAvoidance ************************************************************************************/
/// Class providing methods that realize a JointLimitAvoidance constraint.
template <typename T_PARAMS, typename PRIO = uint32_t>
class JointLimitAvoidance : public ConstraintBase<T_PARAMS, PRIO>
{
    public:
        JointLimitAvoidance(PRIO prio,
                            T_PARAMS constraint_params,
                            CallbackDataMediator& cbdm)
            : ConstraintBase<T_PARAMS, PRIO>(prio, constraint_params, cbdm),
              abs_delta_max_(std::numeric_limits<double>::max()),
              abs_delta_min_(std::numeric_limits<double>::max()),  // max. delta away from min
              rel_max_(1.0),    // 100% rel. range to max limit
              rel_min_(1.0)     // 100% rel. range to min limit
        {}

        virtual ~JointLimitAvoidance()
        {}

        virtual std::string getTaskId() const;

        virtual void calculate();
        virtual Eigen::MatrixXd getTaskJacobian() const;
        virtual Eigen::VectorXd getTaskDerivatives() const;

        virtual double getActivationGain() const;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const;

    private:
        void calcValue();
        void calcDerivativeValue();
        void calcPartialValues();

        double abs_delta_max_;
        double abs_delta_min_;
        double rel_max_;
        double rel_min_;
};
/* END JointLimitAvoidance **************************************************************************************/

/* BEGIN JointLimitAvoidanceMid *********************************************************************************/
/// Class providing methods that realize a JointLimitAvoidanceMid constraint.
template <typename T_PARAMS, typename PRIO = uint32_t>
class JointLimitAvoidanceMid : public ConstraintBase<T_PARAMS, PRIO>
{
    public:
        JointLimitAvoidanceMid(PRIO prio,
                               T_PARAMS constraint_params,
                               CallbackDataMediator& cbdm)
            : ConstraintBase<T_PARAMS, PRIO>(prio, constraint_params, cbdm)
        {}

        virtual ~JointLimitAvoidanceMid()
        {}

        virtual std::string getTaskId() const;

        virtual void calculate();

        virtual double getActivationGain() const;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const;

    private:
        void calcValue();
        void calcDerivativeValue();
        void calcPartialValues();
};
/* END JointLimitAvoidanceMid ***********************************************************************************/

/* BEGIN JointLimitAvoidanceIneq ************************************************************************************/
/// Class providing methods that realize a JointLimitAvoidance constraint based on inequalities.
template <typename T_PARAMS, typename PRIO = uint32_t>
class JointLimitAvoidanceIneq : public ConstraintBase<T_PARAMS, PRIO>
{
    public:
        JointLimitAvoidanceIneq(PRIO prio,
                                T_PARAMS constraint_params,
                                CallbackDataMediator& cbdm)
            : ConstraintBase<T_PARAMS, PRIO>(prio, constraint_params, cbdm),
              abs_delta_max_(std::numeric_limits<double>::max()),
              abs_delta_min_(std::numeric_limits<double>::max()),
              rel_max_(1.0),
              rel_min_(1.0)
        {}

        virtual ~JointLimitAvoidanceIneq()
        {}

        virtual std::string getTaskId() const;
        virtual Eigen::MatrixXd getTaskJacobian() const;
        virtual Eigen::VectorXd getTaskDerivatives() const;

        virtual void calculate();

        virtual double getActivationGain() const;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const;

    private:
        void calcValue();
        void calcDerivativeValue();
        void calcPartialValues();

        double abs_delta_max_;
        double abs_delta_min_;
        double rel_max_;
        double rel_min_;
};
/* END JointLimitAvoidanceIneq **************************************************************************************/

typedef ConstraintsBuilder<uint32_t> ConstraintsBuilder_t;

#include "cob_twist_controller/constraints/constraint_impl.h"   // implementation of templated class

#endif  // COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_H
