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
 *   Implementation of a factory to solve IK problems by using WLN method.
 *
 ****************************************************************/
#ifndef WEIGHTED_LEAST_NORM_SOLVER_FACTORY_H_
#define WEIGHTED_LEAST_NORM_SOLVER_FACTORY_H_

#include <Eigen/Core>
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"
#include "cob_twist_controller/constraint_solvers/factories/solver_factory.h"

/// WeightedLeastNormSolverFactory class implementing SolverFactory
class WeightedLeastNormSolverFactory : public SolverFactory
{
    public:
        virtual ~WeightedLeastNormSolverFactory()
        {
        }

    protected:

        /**
         * Specific implementation of createSolver to create a WeightedLeastNormSolverFactory.
         * The result of the solve method should be equal to the UnconstraintSolver.
         * See base class WeightedLeastNormSolver for more details on params and returns.
         * @returns WeightedLeastNormSolver reference
         */
        virtual ConstraintSolver* createSolver(AugmentedSolverParams &asSolverParams,
                                               Matrix6Xd &jacobianData,
                                               Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed) const;

};

#endif /* WEIGHTED_LEAST_NORM_SOLVER_FACTORY_H_ */


