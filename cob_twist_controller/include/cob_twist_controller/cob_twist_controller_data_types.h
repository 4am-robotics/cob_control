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
#define MAIN_TASK_PRIO 200

const double DAMPING_LIMIT = 1.0e-12; ///< const. value for zero comparison with damping factor
const double ZERO_LIMIT = 1.0e-9;

typedef Eigen::Matrix<double,6,Eigen::Dynamic> t_Matrix6Xd;

typedef Eigen::Matrix<double,6,1> t_Vector6d;
typedef Eigen::Matrix<double,7,1> t_Vector7d;
typedef Eigen::Matrix<double,7,6> t_Matrix76d;

enum DampingMethodTypes {
    NO_DAMPING = 0,
    CONSTANT = 1,
    MANIPULABILITY = 2,
    LEAST_SINGULAR_VALUE = 3,
};

enum HardwareInterfaceTypes {
    VELOCITY_INTERFACE = 0,
    POSITION_INTERFACE = 1,
};

enum KinematicExtensionTypes {
    NO_EXTENSION = 0,
    BASE_ACTIVE = 1,
};

enum SolverTypes {
    DEFAULT_SOLVER = 0,
    WLN,
    GPM,
    TASK_STACK_NO_GPM,
    TASK_STACK_GPM,
    TASK_2ND_PRIO,
    DYN_TASKS_READJ,
};

enum ConstraintTypes
{
    None = 0,
    CA,
    JLA,
    JLA_MID,
    JLA_INEQ,
};

enum ConstraintTypesCA {
    CA_OFF = 0,
    CA_ON,
};

enum ConstraintTypesJLA {
    JLA_OFF = 0,
    JLA_ON,
    JLA_MID_ON,
    JLA_INEQ_ON,
};

struct JointStates
{
    KDL::JntArray current_q_;
    KDL::JntArray last_q_;
    KDL::JntArray current_q_dot_;
    KDL::JntArray last_q_dot_;
};

struct ActiveCartesianDimension {
    ActiveCartesianDimension():
        lin_x(0.0), lin_y(0.0), lin_z(0.0), rot_x(0.0), rot_y(0.0), rot_z(0.0) {}
    
    double lin_x;
    double lin_y;
    double lin_z;
    double rot_x;
    double rot_y;
    double rot_z;
};

struct ObstacleDistanceInfo
{
    double min_distance;
    // Eigen::Vector3d distance_vec;
    t_Vector6d distance_vec;
    t_Vector6d frame_twist;
    std::string frame_id;
    Eigen::Vector3d collision_pnt_vector;
};

struct TwistControllerParams {
    uint8_t dof;
    std::string chain_base_link;
    std::string chain_tip_link;
    
    HardwareInterfaceTypes hardware_interface_type;
    
    bool numerical_filtering;
    DampingMethodTypes damping_method;
    double damping_factor;
    double lambda_max;
    double w_threshold;
    double beta;
    double eps_damping;
    
    SolverTypes solver;
    uint32_t priority_main;

    ConstraintTypesCA constraint_ca;
    uint32_t priority_ca;
    double k_H_ca;
    double damping_ca;

    ConstraintTypesJLA constraint_jla;
    uint32_t priority_jla;
    double k_H_jla;
    double damping_jla;

    double mu;
    double k_H;
    
    double eps_truncation;
    
    bool keep_direction;
    bool enforce_pos_limits;
    bool enforce_vel_limits;
    double tolerance;
    
    // added limits from URDF file
    std::vector<double> limits_max;
    std::vector<double> limits_min;
    std::vector<double> limits_vel;
    
    double max_vel_lin_base;
    double max_vel_rot_base;
    
    bool base_compensation;
    KinematicExtensionTypes kinematic_extension;
    double base_ratio;


    std::vector<std::string> frame_names;
    std::vector<std::string> joints;

    // added a vector to contain all frames of interest for collision checking.
    std::vector<std::string> collision_check_frames;
};

enum EN_ConstraintStates
{
    NORMAL,
    DANGER,
    CRITICAL,
};

class ConstraintState {
    public:

        ConstraintState()
        : current_(NORMAL), previous_(NORMAL), transition_(false)
        {

        }

        inline void setState(EN_ConstraintStates next_state)
        {
            this->transition_ = this->current_ != next_state;
            this->previous_ = this->current_;
            this->current_ = next_state;
        }

        inline EN_ConstraintStates getCurrent() const
        {
            return this->current_;
        }

        inline EN_ConstraintStates getPrevious() const
        {
            return this->previous_;
        }

        inline bool isTransition() const
        {
            return this->transition_;
        }

    private:
        EN_ConstraintStates current_;
        EN_ConstraintStates previous_;
        bool transition_;
};

#endif /* COB_TWIST_CONTROLLER_DATA_TYPES_H_ */
