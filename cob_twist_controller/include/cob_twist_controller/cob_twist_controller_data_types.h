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
 *   Different data types for CobTwistController to be used in other modules
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_DATA_TYPES_H_
#define COB_TWIST_CONTROLLER_DATA_TYPES_H_

#include <vector>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/LU> // necessary to use several methods on EIGEN Matrices.
#include <kdl/chainjnttojacsolver.hpp>

#define MAX_CRIT true
#define MIN_CRIT false

const double DAMPING_LIMIT = 1.0e-12; ///< const. value for zero comparison with damping factor

typedef Eigen::Matrix<double,6,Eigen::Dynamic> t_Matrix6Xd;

typedef Eigen::Matrix<double,6,1> t_Vector6d;

enum DampingMethodTypes {
    NONE = 0,
    CONSTANT = 1,
    MANIPULABILITY = 2,
    LSV = 3,
};

enum InterfaceType {
    VELOCITY = 0,
    POSITION = 1,
};

enum ContraintTypes {
    None = 0,
    WLN = 1,
    WLN_JLA = 2,
    GPM_JLA = 3,
    GPM_JLA_MID = 4,
    GPM_CA = 5,
};

struct ActiveCartesianDimension {
    ActiveCartesianDimension():
        lin_x(0), lin_y(0), lin_z(0), rot_x(0), rot_y(0), rot_z(0) {}
    
    double lin_x;
    double lin_y;
    double lin_z;
    double rot_x;
    double rot_y;
    double rot_z;
};

struct InvDiffKinSolverParams {
    DampingMethodTypes damping_method;
    bool numerical_filtering;
    double damping_factor;
    double lambda_max;
    double w_threshold;
    double beta;
    double eps_damping;
    ContraintTypes constraint;
    double k_H;
    double eps_truncation;
    bool base_compensation;
    bool base_active;
    double base_ratio;


    // added limits from URDF file
    std::vector<double> limits_max;
    std::vector<double> limits_min;
    std::vector<double> limits_vel;
    std::vector<std::string> frame_names;
};

struct ObstacleDistanceInfo
{
    double min_distance;
    Eigen::Vector3d distance_vec;
    std::string frame_id;
};

struct TwistControllerParams {
    uint8_t dof;
    bool base_active;
    bool base_compensation;
    double max_vel_lin;
    double max_vel_rot;
    double max_vel_lin_base;
    double max_vel_rot_base;
    double tolerance;

    bool keep_direction;
    bool enforce_pos_limits;
    bool enforce_vel_limits;

    InterfaceType interface_type;

    // added limits from URDF file
    std::vector<double> limits_vel;
    std::vector<double> limits_min;
    std::vector<double> limits_max;

    // added a vector to contain all frames of interest for collision checking.
    std::vector<std::string> collision_check_frames;
};

#endif /* COB_TWIST_CONTROLLER_DATA_TYPES_H_ */
