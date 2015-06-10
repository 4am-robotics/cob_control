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

#include "cob_twist_controller/callback_data_mediator.h"

/* BEGIN ConstraintsBuilder *************************************************************************************/
/// Class providing a static method to create constraints.
template <typename PRIO = uint32_t>
class ConstraintsBuilder
{
    public:
        static std::set<tConstraintBase> createConstraints(AugmentedSolverParams &augmentedSolverParams,
                                                            const KDL::JntArray& q,
                                                            const Matrix6Xd &jacobianData,
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

        CollisionAvoidance(PRIO prio, const KDL::JntArray& q,
                           boost::shared_ptr<T_PARAMS> constraintParams,
                           KDL::ChainJntToJacSolver& jnt_to_jac) :
            ConstraintBase<T_PARAMS, PRIO>(prio, q, constraintParams), jnt_to_jac_(jnt_to_jac)
        {
            ROS_INFO_STREAM("ctor CollisionAvoidance::constraintParams " << this->constraintParams_->current_distance_.min_distance);
        }

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getSafeRegion() const;
        virtual Eigen::VectorXd getPartialValues() const;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const;

        virtual ~CollisionAvoidance()
        {
        }

    private:
        KDL::ChainJntToJacSolver& jnt_to_jac_;


};
/* END CollisionAvoidance ***************************************************************************************/

/* BEGIN JointLimitAvoidance ************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename T_PARAMS, typename PRIO = uint32_t>
class JointLimitAvoidance : public ConstraintBase<T_PARAMS, PRIO>
{
    public:

        JointLimitAvoidance(PRIO prio,
                            const KDL::JntArray& q,
                            boost::shared_ptr<T_PARAMS> constraintParams)
            : ConstraintBase<T_PARAMS, PRIO>(prio, q, constraintParams)
        {
        }

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getSafeRegion() const;
        virtual Eigen::VectorXd getPartialValues() const;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const;

        double getValue(Eigen::VectorXd steps) const;

        virtual ~JointLimitAvoidance()
        {
        }
};
/* END JointLimitAvoidance **************************************************************************************/

/* BEGIN JointLimitAvoidanceMid *********************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename T_PARAMS, typename PRIO = uint32_t>
class JointLimitAvoidanceMid : public ConstraintBase<T_PARAMS, PRIO>
{
    public:

        JointLimitAvoidanceMid(PRIO prio, const KDL::JntArray& q, boost::shared_ptr<T_PARAMS> constraintParams)
            : ConstraintBase<T_PARAMS, PRIO>(prio, q, constraintParams)
        {
        }

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getSafeRegion() const;
        virtual Eigen::VectorXd getPartialValues() const;
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const;

        virtual ~JointLimitAvoidanceMid()
        {

        }
};
/* END JointLimitAvoidanceMid ***********************************************************************************/


#include "cob_twist_controller/constraints/constraint_impl.h" // implementation of templated class

#endif /* CONSTRAINT_H_ */
