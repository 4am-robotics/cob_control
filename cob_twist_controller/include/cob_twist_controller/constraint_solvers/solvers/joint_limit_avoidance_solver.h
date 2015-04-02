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
 *   This header contains the description of the JLA solver
 *   Implements methods from constraint_solver_base
 *   Special constraint handling.
 *
 ****************************************************************/
#ifndef JOINT_LIMIT_AVOIDANCE_SOLVER_H_
#define JOINT_LIMIT_AVOIDANCE_SOLVER_H_

#include "cob_twist_controller/augmented_solver_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"

class JointLimitAvoidanceSolver : public ConstraintSolver
{
    public:
        virtual Eigen::MatrixXd solve(const Eigen::VectorXd &inCartVelocities, const KDL::JntArray& q, const KDL::JntArray& q_dot) const;

        JointLimitAvoidanceSolver(AugmentedSolverParams &asSolverParams,
                           Matrix6Xd &jacobianData,
                           Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed);
        virtual ~JointLimitAvoidanceSolver();


    private:
        Eigen::MatrixXd calculateWeighting(const KDL::JntArray& q, const KDL::JntArray& q_dot) const;
};

#endif /* JOINT_LIMIT_AVOIDANCE_SOLVER_H_ */
