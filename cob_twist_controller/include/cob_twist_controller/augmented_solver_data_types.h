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
 *   Different data types for AugmentedSolver to be used in other modules
 *
 ****************************************************************/

#ifndef AUGMENTED_SOLVER_DATA_TYPES_H_
#define AUGMENTED_SOLVER_DATA_TYPES_H_

#include <Eigen/Core>
#include <Eigen/LU> // necessary to use several methods on EIGEN Matrices.

struct AugmentedSolverParams {
    int damping_method;
    double eps;
    double damping_factor;
    double lambda0;
    double wt;
    bool JLA_active;
    bool enforce_limits;
    double tolerance;
    bool base_compensation;
    bool base_active;
    double base_ratio;

};

enum DampingMethodTypes {
    MANIPULABILITY = 0,
    CONSTANT = 1,
    TRUNCATION = 2
};

typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;

#endif /* AUGMENTED_SOLVER_DATA_TYPES_H_ */
