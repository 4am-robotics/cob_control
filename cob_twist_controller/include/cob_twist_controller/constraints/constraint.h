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
        static T createConstraintParams(InvDiffKinSolverParams &inv_diff_kin_params,
                                        CallbackDataMediator& data_mediator)
        {
            T params(inv_diff_kin_params);
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
        static std::set<tConstraintBase> createConstraints(InvDiffKinSolverParams &params,
                                                           const JointStates& joint_states,
                                                           const t_Matrix6Xd &jacobian_data,
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
                           const JointStates& joint_states,
                           T_PARAMS constraint_params,
                           KDL::ChainJntToJacSolver& jnt_to_jac) :
            ConstraintBase<T_PARAMS, PRIO>(prio, joint_states, constraint_params), jnt_to_jac_(jnt_to_jac)
        {
            obstacle_avoidance_distance_vec_ = std::numeric_limits<double>::max() * Eigen::Vector3d::Ones();
            //obstacle_avoidance_distance_vec_ = std::numeric_limits<double>::max() * t_Vector6d::Ones();
            minimal_distance_ = std::numeric_limits<double>::max();
        }

        virtual ~CollisionAvoidance()
        {}

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getActivationThreshold() const;
        virtual Eigen::VectorXd getPartialValues();
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const;
        virtual Eigen::MatrixXd getObstacleAvoidancePointJac() const;
        virtual Eigen::Vector3d getDistanceVector() const;


    private:
        KDL::ChainJntToJacSolver& jnt_to_jac_;
        Eigen::Matrix3Xd obstacle_avoidance_point_jac_;
        Eigen::Vector3d obstacle_avoidance_distance_vec_;
        // t_Vector6d obstacle_avoidance_distance_vec_;
        double minimal_distance_;


};
/* END CollisionAvoidance ***************************************************************************************/

/* BEGIN JointLimitAvoidance ************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename T_PARAMS, typename PRIO = uint32_t>
class JointLimitAvoidance : public ConstraintBase<T_PARAMS, PRIO>
{
    public:

        JointLimitAvoidance(PRIO prio,
                            const JointStates& joint_states,
                            T_PARAMS constraint_params)
            : ConstraintBase<T_PARAMS, PRIO>(prio, joint_states, constraint_params)
        {}

        virtual ~JointLimitAvoidance()
        {}

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getActivationThreshold() const;
        virtual Eigen::VectorXd getPartialValues();
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const;
        double getValue(Eigen::VectorXd steps) const;
        virtual Eigen::MatrixXd getObstacleAvoidancePointJac() const;
        virtual Eigen::Vector3d getDistanceVector() const;


};
/* END JointLimitAvoidance **************************************************************************************/

/* BEGIN JointLimitAvoidanceMid *********************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename T_PARAMS, typename PRIO = uint32_t>
class JointLimitAvoidanceMid : public ConstraintBase<T_PARAMS, PRIO>
{
    public:

        JointLimitAvoidanceMid(PRIO prio,
                               const JointStates& joint_states,
                               T_PARAMS constraint_params)
            : ConstraintBase<T_PARAMS, PRIO>(prio, joint_states, constraint_params)
        {}

        virtual ~JointLimitAvoidanceMid()
        {}

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getActivationThreshold() const;
        virtual Eigen::VectorXd getPartialValues();
        virtual double getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const;
        virtual Eigen::MatrixXd getObstacleAvoidancePointJac() const;
        virtual Eigen::Vector3d getDistanceVector() const;
};
/* END JointLimitAvoidanceMid ***********************************************************************************/


#include "cob_twist_controller/constraints/constraint_impl.h" // implementation of templated class

#endif /* CONSTRAINT_H_ */
