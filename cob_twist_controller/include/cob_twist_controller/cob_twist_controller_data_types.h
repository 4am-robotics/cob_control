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

#ifndef COB_TWIST_CONTROLLER_COB_TWIST_CONTROLLER_DATA_TYPES_H
#define COB_TWIST_CONTROLLER_COB_TWIST_CONTROLLER_DATA_TYPES_H

#include <string>
#include <vector>
#include <map>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <kdl/jntarray.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <cob_twist_controller/TwistControllerConfig.h>

#define MAX_CRIT true
#define MIN_CRIT false
#define DEFAULT_CYCLE 0.02
#define ZERO_THRESHOLD 1.0e-9   /// used for numerical 0.0 threshold
#define DIV0_SAFE 1.0e-6        /// used for division in case of DIV/0

enum DampingMethodTypes
{
    NO_DAMPING = cob_twist_controller::TwistController_NO_DAMPING,
    CONSTANT = cob_twist_controller::TwistController_CONSTANT,
    MANIPULABILITY = cob_twist_controller::TwistController_MANIPULABILITY,
    LEAST_SINGULAR_VALUE = cob_twist_controller::TwistController_LEAST_SINGULAR_VALUE,
    SIGMOID = cob_twist_controller::TwistController_SIGMOID,
};

enum KinematicExtensionTypes
{
    NO_EXTENSION = cob_twist_controller::TwistController_NO_EXTENSION,
    BASE_COMPENSATION = cob_twist_controller::TwistController_BASE_COMPENSATION,
    BASE_ACTIVE = cob_twist_controller::TwistController_BASE_ACTIVE,
    COB_TORSO = cob_twist_controller::TwistController_COB_TORSO,
    LOOKAT = cob_twist_controller::TwistController_LOOKAT
};

enum SolverTypes
{
    DEFAULT_SOLVER = cob_twist_controller::TwistController_DEFAULT_SOLVER,
    WLN = cob_twist_controller::TwistController_WLN,
    GPM = cob_twist_controller::TwistController_GPM,
    STACK_OF_TASKS = cob_twist_controller::TwistController_STACK_OF_TASKS,
    TASK_2ND_PRIO = cob_twist_controller::TwistController_TASK_2ND_PRIO,
    UNIFIED_JLA_SA = cob_twist_controller::TwistController_UNIFIED_JLA_SA,
};

enum ConstraintTypesCA
{
    CA_OFF = cob_twist_controller::TwistController_CA_OFF,
    CA_ON = cob_twist_controller::TwistController_CA,
};

enum ConstraintTypesJLA
{
    JLA_OFF = cob_twist_controller::TwistController_JLA_OFF,
    JLA_ON = cob_twist_controller::TwistController_JLA,
    JLA_MID_ON = cob_twist_controller::TwistController_JLA_MID,
    JLA_INEQ_ON = cob_twist_controller::TwistController_JLA_INEQ,
};

enum ConstraintTypes
{
    None,
    CA,
    JLA,
    JLA_MID,
    JLA_INEQ,
};

enum LookatAxisTypes
{
    X_POSITIVE,
    Y_POSITIVE,
    Z_POSITIVE,
    X_NEGATIVE,
    Y_NEGATIVE,
    Z_NEGATIVE,
};

struct LookatOffset
{
    LookatOffset() :
        lookat_axis_type(X_POSITIVE),
        translation_x(0.0),
        translation_y(0.0),
        translation_z(0.0),
        rotation_x(0.0),
        rotation_y(0.0),
        rotation_z(0.0),
        rotation_w(1.0)
    {}

    LookatAxisTypes lookat_axis_type;
    double translation_x;
    double translation_y;
    double translation_z;
    double rotation_x;
    double rotation_y;
    double rotation_z;
    double rotation_w;
};

struct JointStates
{
    KDL::JntArray current_q_;
    KDL::JntArray last_q_;
    KDL::JntArray current_q_dot_;
    KDL::JntArray last_q_dot_;
};

struct ActiveCartesianDimension
{
    ActiveCartesianDimension():
        lin_x(0.0), lin_y(0.0), lin_z(0.0), rot_x(0.0), rot_y(0.0), rot_z(0.0) {}

    double lin_x;
    double lin_y;
    double lin_z;
    double rot_x;
    double rot_y;
    double rot_z;
};

struct ObstacleDistanceData
{
    double min_distance;
    Eigen::Vector3d frame_vector;
    Eigen::Vector3d nearest_point_frame_vector;
    Eigen::Vector3d nearest_point_obstacle_vector;
};

struct ConstraintThresholds
{
    double activation;
    double activation_with_buffer;
    double critical;
    double activation_position_threshold_jla;
    double activation_speed_threshold_jla;
};

struct LimiterParams
{
    LimiterParams() :
        keep_direction(true),
        enforce_input_limits(true),
        enforce_pos_limits(true),
        enforce_vel_limits(true),
        enforce_acc_limits(false),
        limits_tolerance(5.0),
        max_lin_twist(0.5),
        max_rot_twist(0.5),
        max_vel_lin_base(0.5),
        max_vel_rot_base(0.5)
    {}

    bool keep_direction;
    bool enforce_input_limits;
    bool enforce_pos_limits;
    bool enforce_vel_limits;
    bool enforce_acc_limits;
    double limits_tolerance;
    double max_lin_twist;
    double max_rot_twist;
    double max_vel_lin_base;
    double max_vel_rot_base;

    std::vector<double> limits_max;
    std::vector<double> limits_min;
    std::vector<double> limits_vel;
    std::vector<double> limits_acc;
};

struct TwistControllerParams
{
    TwistControllerParams() :
        dof(0),
        controller_interface(""),
        integrator_smoothing(0.2),

        numerical_filtering(false),
        damping_method(SIGMOID),
        damping_factor(0.2),
        lambda_max(0.01),
        w_threshold(0.01),
        beta(0.005),
        slope_damping(0.05),
        eps_damping(0.003),
        eps_truncation(0.001),
        solver(GPM),
        priority_main(500),
        k_H(1.0),

        constraint_jla(JLA_OFF),
        priority_jla(50),
        k_H_jla(-10.0),
        damping_jla(0.05),
        damping_speed_jla(0.005),

        constraint_ca(CA_OFF),
        priority_ca(100),
        damping_ca(0.000001),
        k_H_ca(2.0),

        kinematic_extension(NO_EXTENSION),
        extension_ratio(0.0)
    {
        this->thresholds_ca.activation = 0.1;
        this->thresholds_ca.critical = 0.025;
        this->thresholds_ca.activation_with_buffer = this->thresholds_ca.activation * 1.5;  // best experienced value

        this->thresholds_jla.activation = 0.1;
        this->thresholds_jla.critical = 0.05;
        this->thresholds_jla.activation_with_buffer = this->thresholds_jla.activation * 4.0;  // best experienced value
        this->thresholds_jla.activation_position_threshold_jla = 0.5;
        this->thresholds_jla.activation_speed_threshold_jla = 1.0;
    }

    uint8_t dof;
    std::string chain_base_link;
    std::string chain_tip_link;

    std::string controller_interface;
    double integrator_smoothing;

    bool numerical_filtering;
    DampingMethodTypes damping_method;
    double damping_factor;
    double lambda_max;
    double w_threshold;
    double beta;
    double slope_damping;
    double eps_damping;
    double eps_truncation;

    SolverTypes solver;
    uint32_t priority_main;
    double k_H;

    ConstraintTypesCA constraint_ca;
    uint32_t priority_ca;
    double k_H_ca;
    double damping_ca;
    ConstraintThresholds thresholds_ca;

    ConstraintTypesJLA constraint_jla;
    uint32_t priority_jla;
    double k_H_jla;
    double damping_jla;
    double damping_speed_jla;
    ConstraintThresholds thresholds_jla;

    LimiterParams limiter_params;

    KinematicExtensionTypes kinematic_extension;
    LookatOffset lookat_offset;
    double extension_ratio;

    std::vector<std::string> frame_names;
    std::vector<std::string> joints;

    // vector of links of the chain to be considered for collision avoidance
    std::vector<std::string> collision_check_links;
};

enum EN_ConstraintStates
{
    NORMAL = 0,
    DANGER,
    CRITICAL,
};

class ConstraintState
{
    public:
        ConstraintState()
        : current_(NORMAL), previous_(NORMAL), transition_(false)
        {}

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

typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd_t;
typedef Eigen::Matrix<double, 6, 1> Vector6d_t;
typedef std::map<std::string, std::vector<ObstacleDistanceData> > ObstacleDistancesInfo_t;

#endif  // COB_TWIST_CONTROLLER_COB_TWIST_CONTROLLER_DATA_TYPES_H
