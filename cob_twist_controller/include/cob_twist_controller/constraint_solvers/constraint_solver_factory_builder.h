/*
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
 *   This header contains the description of a class providing a static method to create constraint solver factory objects.
 *
 ****************************************************************/
#ifndef CONSTRAINT_SOLVER_FACTORY_BUILDER_H_
#define CONSTRAINT_SOLVER_FACTORY_BUILDER_H_

#include <Eigen/Core>
#include <Eigen/SVD>
#include <kdl/jntarray.hpp>
#include "cob_twist_controller/augmented_solver_data_types.h"

/// Static class providing a single method for creation of damping method, solver and starting the solving of the IK problem.
class ConstraintSolverFactoryBuilder
{
    public:

        /**
         * Calculation of new joint velocities according to current joint positions and cartesian velocities.
         * @param asSolverParams References the augmented solver parameters.
         * @param jacobianData References the current Jacobian (matrix data only).
         * @param jacobianDataTransposed References the current Jacobian transpose (matrix data only).
         * @param inCartVelocities The input velocities vector (in cartesian space).
         * @param q The current joint positions.
         * @param last_q_dot The last joint velocities.
         * @return The calculated new joint velocities in (m x 1)-Matrix.
         */
        static Eigen::MatrixXd calculateJointVelocities(AugmentedSolverParams &asSolverParams,
                                                       Matrix6Xd &jacobianData,
                                                       Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed,
                                                       const Eigen::VectorXd &inCartVelocities,
                                                       const KDL::JntArray& q,
                                                       const KDL::JntArray& last_q_dot);

    private:
        ConstraintSolverFactoryBuilder() {}
        ~ConstraintSolverFactoryBuilder() {}
};

#endif /* CONSTRAINT_SOLVER_FACTORY_BUILDER_H_ */
