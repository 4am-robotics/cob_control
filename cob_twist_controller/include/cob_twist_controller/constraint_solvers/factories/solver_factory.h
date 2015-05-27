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

/// Interface definition to support generic usage of the solver factory without specifying a typename in prior.
class ISolverFactory
{
    public:
        virtual Eigen::MatrixXd calculateJointVelocities(AugmentedSolverParams &asParams,
                                                         Matrix6Xd &jacobianData,
                                                         const Eigen::VectorXd &inCartVelocities,
                                                         const KDL::JntArray& q,
                                                         const KDL::JntArray& last_q_dot,
                                                         double dampingFactor) const = 0;

        virtual ~ISolverFactory() {}
};

/// Abstract base class defining interfaces for the creation of a specific solver.
template <typename T>
class SolverFactory : public ISolverFactory
{
    public:

        /**
         * The base calculation method to calculate joint velocities out of input velocities (cartesian space).
         * Creates a solver according to implemented createSolver-method (in subclass).
         * Use the specialized solve-method to calculate new joint velocities.
         * @param asParams References the augmented solver parameters.
         * @param jacobianData References the current Jacobian (matrix data only).
         * @param jacobianDataTransposed References the current Jacobian transpose (matrix data only).
         * @param inCartVelocities The input velocities vector (in cartesian space).
         * @param q The current joint positions.
         * @param last_q_dot The last joint velocities.
         * @param dampingFactor The damping factor corresponding to damping method.
         * @return Joint velocities in a (m x 1)-Matrix.
         */
        Eigen::MatrixXd calculateJointVelocities(AugmentedSolverParams &asParams,
                                                 Matrix6Xd &jacobianData,
                                                 const Eigen::VectorXd &inCartVelocities,
                                                 const KDL::JntArray& q,
                                                 const KDL::JntArray& last_q_dot,
                                                 double dampingFactor) const
        {
            T* cs = this->createSolver(asParams, jacobianData);
            cs->setDampingFactor(dampingFactor);
            Eigen::MatrixXd new_q_dot = cs->solve(inCartVelocities, q, last_q_dot);
            delete cs;
            cs = NULL;
            return new_q_dot;
        }

    protected:

        /**
         * The interface method to create a specific solver to solve the inverse kinematics problem.
         * @param asParams References the augmented solver parameters.
         * @param jacobianData References the current Jacobian (matrix data only).
         * @param jacobianDataTransposed References the current Jacobian transpose (matrix data only).
         * @return A specific solver.
         */
        T* createSolver(AugmentedSolverParams &asParams,
                                               Matrix6Xd &jacobianData) const
        {
            return new T(asParams, jacobianData);
        }

};

#endif /* SOLVER_FACTORY_H_ */
