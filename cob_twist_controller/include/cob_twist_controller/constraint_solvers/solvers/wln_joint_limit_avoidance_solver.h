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

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/weighted_least_norm_solver.h"

/// Implementation of ConstraintSolver to solve inverse kinematics with joint limit avoidance
/// Uses solve method of the WeightedLeastNormSolver
class WLN_JointLimitAvoidanceSolver : public WeightedLeastNormSolver
{
    public:

        WLN_JointLimitAvoidanceSolver(const TwistControllerParams& params)
                                  : WeightedLeastNormSolver(params)
        {

        }

        virtual ~WLN_JointLimitAvoidanceSolver()
        {

        }


    private:

        /**
         * Helper method that calculates a weighting for the Jacobian to adapt the impact on joint velocities.
         * Overridden from base class WLNSolver
         * @param q The current joint positions.
         * @param q_dot The current joint velocities.
         * @return Diagonal weighting matrix that adapts the Jacobian.
         */
        virtual Eigen::MatrixXd calculateWeighting(const JointStates& joint_states);
};

#endif /* JOINT_LIMIT_AVOIDANCE_SOLVER_H_ */
