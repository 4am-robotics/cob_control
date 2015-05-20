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
#include <vector>
#include <stdint.h>

enum DampingMethodTypes {
    NONE = 0,
    CONSTANT = 1,
    MANIPULABILITY = 2,
    LSV = 3,
};

enum ContraintTypes {
    None = 0,
    WLN = 1,
    WLN_JLA = 2,
};

struct AugmentedSolverParams {
    DampingMethodTypes damping_method;
    bool numerical_filtering;
    double damping_factor;
    double lambda_max;
    double w_threshold;
    double beta;
    double eps_damping;
    ContraintTypes constraint;
    double eps_truncation;
    double p_gain;
    bool base_compensation;
    bool base_active;
    double base_ratio;

    // added limits from URDF file
    std::vector<double> limits_max;
    std::vector<double> limits_min;
};

const double DAMPING_LIMIT = 1.0e-12; ///< const. value for zero comparison with damping factor

typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;

#endif /* AUGMENTED_SOLVER_DATA_TYPES_H_ */
