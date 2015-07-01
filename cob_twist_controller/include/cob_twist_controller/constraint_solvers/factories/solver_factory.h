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
 * \date Date of creation: March, 2015
 *
 * \brief
 *   This header contains the interface description to create solvers
 *
 ****************************************************************/
#ifndef SOLVER_FACTORY_H_
#define SOLVER_FACTORY_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/jntarray.hpp>

#include "cob_twist_controller/damping_methods/damping_base.h"
#include "cob_twist_controller/constraints/constraint_base.h"

/// Interface definition to support generic usage of the solver factory without specifying a typename in prior.
class ISolverFactory
{
    public:
        virtual Eigen::MatrixXd calculateJointVelocities(t_Matrix6Xd& jacobian_data,
                                                         const t_Vector6d& in_cart_velocities,
                                                         const JointStates& joint_states,
                                                         boost::shared_ptr<DampingBase>& damping_method,
                                                         std::set<tConstraintBase>& constraints) const = 0;

        virtual ~ISolverFactory() {}
};

/// Abstract base class defining interfaces for the creation of a specific solver.
template <typename T>
class SolverFactory : public ISolverFactory
{
    public:

        SolverFactory(const TwistControllerParams& params)
        {
            constraint_solver_.reset(new T(params));
        }

        ~SolverFactory()
        {
            constraint_solver_.reset();
        }

        /**
         * The base calculation method to calculate joint velocities out of input velocities (cartesian space).
         * Creates a solver according to implemented createSolver-method (in subclass).
         * Use the specialized solve-method to calculate new joint velocities.
         * @param params References the inv. diff. kin. solver parameters.
         * @param jacobian_data References the current Jacobian (matrix data only).
         * @param in_cart_velocities The input velocities vector (in cartesian space).
         * @param joint_states The joint states with history.
         * @param damping_method The damping method.
         * @return Joint velocities in a (m x 1)-Matrix.
         */
        Eigen::MatrixXd calculateJointVelocities(t_Matrix6Xd& jacobian_data,
                                                 const t_Vector6d& in_cart_velocities,
                                                 const JointStates& joint_states,
                                                 boost::shared_ptr<DampingBase>& damping_method,
                                                 std::set<tConstraintBase>& constraints) const
        {
            constraint_solver_->setJacobianData(jacobian_data);
            constraint_solver_->setConstraints(constraints);
            constraint_solver_->setDamping(damping_method);
            Eigen::MatrixXd new_q_dot = constraint_solver_->solve(in_cart_velocities, joint_states);
            return new_q_dot;
        }

    private:
        boost::shared_ptr<T> constraint_solver_;
};

#endif /* SOLVER_FACTORY_H_ */
