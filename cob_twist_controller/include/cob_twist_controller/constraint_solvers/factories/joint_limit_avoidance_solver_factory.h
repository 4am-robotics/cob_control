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
 * \date Date of creation: April, 2015
 *
 * \brief
 *   Implementation of a factory to solve unconstrained problems.
 *
 ****************************************************************/
#ifndef JOINT_LIMIT_AVOIDANCE_SOLVER_FACTORY_H_
#define JOINT_LIMIT_AVOIDANCE_SOLVER_FACTORY_H_

#include <Eigen/Core>
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"
#include "cob_twist_controller/constraint_solvers/factories/solver_factory.h"

/// JointLimitAvoidanceSolverFactory class implementing SolverFactory
class JointLimitAvoidanceSolverFactory : public SolverFactory
{
    public:
        virtual ~JointLimitAvoidanceSolverFactory()
        {
        }

    protected:

        /**
         * Specific implementation of createSolver to create an JointLimitAvoidanceSolver.
         * See base class ConstraintSolver for more details on params and returns.
         * @returns JointLimitAvoidanceSolver reference
         */
        virtual ConstraintSolver* createSolver(AugmentedSolverParams &asSolverParams,
                                               Matrix6Xd &jacobianData,
                                               Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed) const;

};

#endif /* JOINT_LIMIT_AVOIDANCE_SOLVER_FACTORY_H_ */


