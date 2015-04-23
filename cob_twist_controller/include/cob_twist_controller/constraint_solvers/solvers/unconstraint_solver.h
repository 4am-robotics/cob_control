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
 *   This header contains the description of the unconstraint solver
 *   Implements methods from constraint_solver_base
 *
 ****************************************************************/
#ifndef UNCONSTRAINT_SOLVER_H_
#define UNCONSTRAINT_SOLVER_H_

#include "cob_twist_controller/augmented_solver_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

class UnconstraintSolver : public ConstraintSolver
{
    public:

        /**
         * Specific implementation of solve-method to solve IK problem without any constraints.
         * See base class ConstraintSolver for more details on params and returns.
         */
        virtual Eigen::MatrixXd solve(const Eigen::VectorXd &inCartVelocities, const KDL::JntArray& q, const KDL::JntArray& last_q_dot) const;

        UnconstraintSolver(AugmentedSolverParams &asParams,
                           Matrix6Xd &jacobianData)
                           : ConstraintSolver(asParams,
                                              jacobianData)
        {

        }

        virtual ~UnconstraintSolver()
        {

        }
};

#endif /* UNCONSTRAINT_SOLVER_H_ */
