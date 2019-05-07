/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


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
};

struct ConstraintParams
{
    uint32_t priority;
    double k_H;
    double damping;
    ConstraintThresholds thresholds;
};

enum ConstraintTypes
{
    CA,
    JLA,
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

struct UJSSolverParams
{
    UJSSolverParams() :
        sigma(0.05),
        sigma_speed(0.005),
        delta_pos(0.5),
        delta_speed(1.0)
    {}

    double sigma;
    double sigma_speed;
    double delta_pos;
    double delta_speed;
};

struct TwistControllerParams
{
    TwistControllerParams() :
        dof(0),
        controller_interface(""),
        integrator_smoothing(0.2),

        numerical_filtering(false),
        damping_method(SIGMOID),
        damping_factor(0.01),
        lambda_max(0.001),
        w_threshold(0.001),
        beta(0.005),
        slope_damping(0.05),
        eps_damping(0.003),
        eps_truncation(0.001),

        solver(GPM),
        priority_main(500),
        k_H(1.0),

        constraint_jla(JLA_ON),
        constraint_ca(CA_ON),

        kinematic_extension(NO_EXTENSION),
        extension_ratio(0.0)
    {
        ConstraintParams cp_ca;
        cp_ca.priority = 100;
        cp_ca.k_H = 2.0;
        cp_ca.damping = 0.000001;
        cp_ca.thresholds.activation = 0.1;
        cp_ca.thresholds.critical = 0.025;
        cp_ca.thresholds.activation_with_buffer = cp_ca.thresholds.activation * 1.5;  // best experienced value
        constraint_params.insert(std::pair<ConstraintTypes, ConstraintParams>(CA, cp_ca));

        ConstraintParams cp_jla;
        cp_jla.priority = 50;
        cp_jla.k_H = -10.0;
        cp_jla.damping = 0.000001;
        cp_jla.thresholds.activation = 0.1;
        cp_jla.thresholds.critical = 0.05;
        cp_jla.thresholds.activation_with_buffer = cp_jla.thresholds.activation * 4.0;  // best experienced value
        constraint_params.insert(std::pair<ConstraintTypes, ConstraintParams>(JLA, cp_jla));
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
    ConstraintTypesJLA constraint_jla;
    std::map<ConstraintTypes, ConstraintParams> constraint_params;

    UJSSolverParams ujs_solver_params;
    LimiterParams limiter_params;

    KinematicExtensionTypes kinematic_extension;
    std::string lookat_pointing_frame;
    LookatOffset lookat_offset;
    double extension_ratio;

    std::vector<std::string> frame_names;
    std::vector<std::string> joints;

    // vector of links of the chain to be considered for collision avoidance
    std::vector<std::string> collision_check_links;

    void from_config(cob_twist_controller::TwistControllerConfig config)
    {
        numerical_filtering = config.numerical_filtering;
        damping_method = static_cast<DampingMethodTypes>(config.damping_method);
        damping_factor = config.damping_factor;
        lambda_max = config.lambda_max;
        w_threshold = config.w_threshold;
        slope_damping = config.slope_damping;
        beta = config.beta;
        eps_damping = config.eps_damping;
        eps_truncation = config.eps_truncation;

        solver = static_cast<SolverTypes>(config.solver);
        priority_main = config.priority;
        k_H = config.k_H;

        constraint_jla = static_cast<ConstraintTypesJLA>(config.constraint_jla);
        constraint_ca = static_cast<ConstraintTypesCA>(config.constraint_ca);

        ConstraintParams cp_jla;
        cp_jla.priority = config.priority_jla;
        cp_jla.k_H = config.k_H_jla;
        cp_jla.damping = config.damping_jla;
        const double activation_jla_in_percent = config.activation_threshold_jla;
        const double activation_buffer_jla_in_percent = config.activation_buffer_jla;
        const double critical_jla_in_percent = config.critical_threshold_jla;
        cp_jla.thresholds.activation = activation_jla_in_percent / 100.0;
        cp_jla.thresholds.critical = critical_jla_in_percent / 100.0;
        cp_jla.thresholds.activation_with_buffer = cp_jla.thresholds.activation * (1.0 + activation_buffer_jla_in_percent / 100.0);
        constraint_params[JLA] = cp_jla;

        ConstraintParams cp_ca;
        cp_ca.priority = config.priority_ca;
        cp_ca.k_H = config.k_H_ca;
        cp_ca.damping = config.damping_ca;
        const double activaton_buffer_ca_in_percent = config.activation_buffer_ca;
        cp_ca.thresholds.activation = config.activation_threshold_ca;  // in [m]
        cp_ca.thresholds.critical = config.critical_threshold_ca;  // in [m]
        cp_ca.thresholds.activation_with_buffer = cp_ca.thresholds.activation * (1.0 + activaton_buffer_ca_in_percent / 100.0);
        constraint_params[CA] = cp_ca;

        ujs_solver_params.sigma = config.sigma;
        ujs_solver_params.sigma_speed = config.sigma_speed;
        ujs_solver_params.delta_pos = config.delta_pos;
        ujs_solver_params.delta_speed = config.delta_speed;

        limiter_params.keep_direction = config.keep_direction;
        limiter_params.enforce_input_limits = config.enforce_input_limits;
        limiter_params.enforce_pos_limits = config.enforce_pos_limits;
        limiter_params.enforce_vel_limits = config.enforce_vel_limits;
        limiter_params.enforce_acc_limits = config.enforce_acc_limits;
        limiter_params.limits_tolerance = config.limits_tolerance;
        limiter_params.max_lin_twist = config.max_lin_twist;
        limiter_params.max_rot_twist = config.max_rot_twist;
        limiter_params.max_vel_lin_base = config.max_vel_lin_base;
        limiter_params.max_vel_rot_base = config.max_vel_rot_base;

        kinematic_extension = static_cast<KinematicExtensionTypes>(config.kinematic_extension);
        extension_ratio = config.extension_ratio;
    }

    void to_config(cob_twist_controller::TwistControllerConfig& config)
    {
        config.numerical_filtering = numerical_filtering;
        config.damping_method = damping_method;
        config.damping_factor = damping_factor;
        config.lambda_max = lambda_max;
        config.w_threshold = w_threshold;
        config.slope_damping = slope_damping;
        config.beta = beta;
        config.eps_damping = eps_damping;
        config.eps_truncation = eps_truncation;

        config.solver = solver;
        config.priority = priority_main;
        config.k_H = k_H;

        config.constraint_jla = config.constraint_jla;
        config.constraint_ca = constraint_ca;

        config.priority_jla = constraint_params[JLA].priority;
        config.k_H_jla = constraint_params[JLA].k_H;
        config.damping_jla = constraint_params[JLA].damping;
        config.activation_threshold_jla = constraint_params[JLA].thresholds.activation * 100.0;
        config.critical_threshold_jla = constraint_params[JLA].thresholds.critical * 100.0;
        config.activation_buffer_jla = (constraint_params[JLA].thresholds.activation_with_buffer/constraint_params[JLA].thresholds.activation - 1.0)*100.0;

        config.priority_ca = constraint_params[CA].priority;
        config.k_H_ca = constraint_params[CA].k_H;
        config.damping_ca = constraint_params[CA].damping;
        config.activation_threshold_ca = constraint_params[CA].thresholds.activation;
        config.critical_threshold_ca = constraint_params[CA].thresholds.critical;
        config.activation_buffer_ca = (constraint_params[CA].thresholds.activation_with_buffer/constraint_params[CA].thresholds.activation - 1.0)*100.0;

        config.sigma = ujs_solver_params.sigma;
        config.sigma_speed = ujs_solver_params.sigma_speed;
        config.delta_pos = ujs_solver_params.delta_pos;
        config.delta_speed = ujs_solver_params.delta_speed;

        config.keep_direction = limiter_params.keep_direction;
        config.enforce_input_limits = limiter_params.enforce_input_limits;
        config.enforce_pos_limits = limiter_params.enforce_pos_limits;
        config.enforce_vel_limits = limiter_params.enforce_vel_limits;
        config.enforce_acc_limits = limiter_params.enforce_acc_limits;
        config.limits_tolerance = limiter_params.limits_tolerance;
        config.max_lin_twist = limiter_params.max_lin_twist;
        config.max_rot_twist = limiter_params.max_rot_twist;
        config.max_vel_lin_base = limiter_params.max_vel_lin_base;
        config.max_vel_rot_base = limiter_params.max_vel_rot_base;

        config.kinematic_extension = kinematic_extension;
        config.extension_ratio = extension_ratio;
    }
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
