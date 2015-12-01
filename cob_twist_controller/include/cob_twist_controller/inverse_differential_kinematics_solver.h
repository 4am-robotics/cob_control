/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
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
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This package provides the definitions of an inverse kinematics solver.
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_INVERSE_DIFFERENTIAL_KINEMATICS_SOLVER_H
#define COB_TWIST_CONTROLLER_INVERSE_DIFFERENTIAL_KINEMATICS_SOLVER_H

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/callback_data_mediator.h"
#include "cob_twist_controller/limiters/limiter.h"
#include "cob_twist_controller/kinematic_extensions/kinematic_extension_builder.h"
#include "cob_twist_controller/constraint_solvers/constraint_solver_factory.h"
#include "cob_twist_controller/task_stack/task_stack_controller.h"

/**
* Implementation of a inverse velocity kinematics algorithm based
* on the generalize pseudo inverse to calculate the velocity
* transformation from Cartesian to joint space of a general
* KDL::Chain. It uses a svd-calculation based on householders
* rotations.
*
* @ingroup KinematicFamily
*/
class InverseDifferentialKinematicsSolver
{
public:
    /**
     * Constructor of the solver
     *
     * @param chain the chain to calculate the inverse velocity
     * kinematics for
     *
     */
    InverseDifferentialKinematicsSolver(const TwistControllerParams& params, const KDL::Chain& chain, CallbackDataMediator& data_mediator) :
        params_(params),
        limiter_params_(params_.limiter_params),
        chain_(chain),
        jac_(chain_.getNrOfJoints()),
        jnt2jac_(chain_),
        fk_solver_vel_(chain_),
        callback_data_mediator_(data_mediator),
        constraint_solver_factory_(data_mediator, jnt2jac_, fk_solver_vel_, task_stack_controller_)
    {
        this->kinematic_extension_.reset(KinematicExtensionBuilder::createKinematicExtension(this->params_));
        this->limiter_params_ = this->kinematic_extension_->adjustLimiterParams(this->limiter_params_);

        this->limiters_.reset(new LimiterContainer(this->limiter_params_));
        this->limiters_->init();
    }

    virtual ~InverseDifferentialKinematicsSolver()
    {
        this->limiters_.reset();
        this->kinematic_extension_.reset();
    };

    /** CartToJnt for chain using SVD considering KinematicExtensions and various DampingMethods **/
    virtual int CartToJnt(const JointStates& joint_states,
                          const KDL::Twist& v_in,
                          KDL::JntArray& qdot_out);

    void resetAll(TwistControllerParams params);

private:
    const KDL::Chain chain_;
    KDL::Jacobian jac_;
    KDL::ChainFkSolverVel_recursive fk_solver_vel_;
    KDL::ChainJntToJacSolver jnt2jac_;
    TwistControllerParams params_;
    LimiterParams limiter_params_;
    CallbackDataMediator& callback_data_mediator_;
    boost::shared_ptr<LimiterContainer> limiters_;
    boost::shared_ptr<KinematicExtensionBase> kinematic_extension_;
    ConstraintSolverFactory constraint_solver_factory_;

    TaskStackController_t task_stack_controller_;
};

#endif  // COB_TWIST_CONTROLLER_INVERSE_DIFFERENTIAL_KINEMATICS_SOLVER_H
