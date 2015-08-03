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

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraints/constraint_base.h"
#include "cob_twist_controller/callback_data_mediator.h"

/* BEGIN ConstraintParamFactory *********************************************************************************/
/// Creates constraint parameters and fills them with the values provided by CallbackDataMediator.
template
<typename T>
class ConstraintParamFactory
{
    public:
        static T createConstraintParams(const TwistControllerParams& twist_controller_params,
                                        CallbackDataMediator& data_mediator)
        {
            T params(twist_controller_params);
            data_mediator.fill(params);
            return params;
        }

    private:
        ConstraintParamFactory()
        {}
};
/* END ConstraintParamFactory ***********************************************************************************/

/* BEGIN ConstraintsBuilder *************************************************************************************/
/// Class providing a static method to create constraints.
template <typename PRIO = uint32_t>
class ConstraintsBuilder
{
    public:
        static std::set<ConstraintBase_t> createConstraints(const TwistControllerParams& params,
                                                           KDL::ChainJntToJacSolver& jnt_to_jac_,
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
                           KDL::ChainJntToJacSolver& jnt_to_jac) :
            ConstraintBase<T_PARAMS, PRIO>(prio, constraint_params, cbdm), jnt_to_jac_(jnt_to_jac)
        {
        }

        virtual ~CollisionAvoidance()
        {}

        virtual std::string getTaskId() const;

        virtual double getCriticalValue() const;

        virtual void calculate();

        virtual double getActivationGain() const;

        virtual double getActivationThreshold() const;

        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution,
                                              const Eigen::MatrixXd& homogeneous_solution) const;

        virtual ConstraintTypes getType() const;

        virtual Eigen::MatrixXd getTaskJacobian() const;

        virtual Eigen::VectorXd getTaskDerivatives() const;

        virtual Task_t createTask();


    private:
        double calcValue();
        double calcDerivativeValue();
        Eigen::VectorXd calcPartialValues();

        KDL::ChainJntToJacSolver& jnt_to_jac_;


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
            : ConstraintBase<T_PARAMS, PRIO>(prio, constraint_params, cbdm)
        {}

        virtual ~JointLimitAvoidance()
        {}

        virtual std::string getTaskId() const;
        virtual void calculate();
        virtual double getActivationGain() const;
        virtual double getActivationThreshold() const;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const;
        virtual ConstraintTypes getType() const;
        virtual Eigen::MatrixXd getTaskJacobian() const;
        virtual Eigen::VectorXd getTaskDerivatives() const;
        virtual Task_t createTask();


    private:
        double calcValue();
        double calcDerivativeValue();
        Eigen::VectorXd calcPartialValues();

        std::vector<uint8_t> active_idx_;
        Eigen::VectorXd values_;
        Eigen::VectorXd last_values_;
        Eigen::VectorXd derivative_values_;
};
/* END JointLimitAvoidance **************************************************************************************/

/* BEGIN JointLimitAvoidanceMid *********************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
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
        virtual double getActivationThreshold() const;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const;
        virtual ConstraintTypes getType() const;

    private:
        double calcValue();
        double calcDerivativeValue();
        Eigen::VectorXd calcPartialValues();
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
        {
        }

        virtual ~JointLimitAvoidanceIneq()
        {}

        virtual std::string getTaskId() const;
        virtual void calculate();
        virtual double getActivationGain() const;
        virtual double getActivationThreshold() const;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const;
        virtual ConstraintTypes getType() const;
        virtual Eigen::MatrixXd getTaskJacobian() const;
        virtual Eigen::VectorXd getTaskDerivatives() const;
        virtual Task_t createTask();

    private:
        double calcValue();
        double calcDerivativeValue();
        Eigen::VectorXd calcPartialValues();

        double abs_delta_max_;
        double abs_delta_min_;
        double rel_max_;
        double rel_min_;
};
/* END JointLimitAvoidanceIneq **************************************************************************************/

typedef ConstraintsBuilder<uint32_t> ConstraintsBuilder_t;

#include "cob_twist_controller/constraints/constraint_impl.h" // implementation of templated class

#endif /* CONSTRAINT_H_ */
