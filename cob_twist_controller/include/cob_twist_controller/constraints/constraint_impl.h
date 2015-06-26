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
 * \date Date of creation: May, 2015
 *
 * \brief
 *   Implementation of several constraints
 *
 ****************************************************************/

#ifndef CONSTRAINT_IMPL_H_
#define CONSTRAINT_IMPL_H_

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <ros/ros.h>

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>

#include "cob_twist_controller/constraints/constraint.h"
#include "cob_twist_controller/constraints/constraint_params.h"

#include "cob_twist_controller/damping_methods/damping.h"
#include "cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h"




/* BEGIN ConstraintsBuilder *************************************************************************************/
/**
 * Static builder method to create damping methods dependent on parameterization.
 */
template <typename PRIO>
std::set<tConstraintBase> ConstraintsBuilder<PRIO>::createConstraints(InvDiffKinSolverParams &inv_diff_kin_solver_params,
                                                                       const JointStates& joint_states,
                                                                       const t_Matrix6Xd &jacobian_data,
                                                                       KDL::ChainJntToJacSolver& jnt_to_jac,
                                                                       CallbackDataMediator& data_mediator)
{
    std::set<tConstraintBase> constraints;
    if (GPM_JLA == inv_diff_kin_solver_params.constraint)
    {
        typedef JointLimitAvoidance<ConstraintParamsJLA, PRIO> tJla;
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(inv_diff_kin_solver_params, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<tJla > jla(new tJla(100, joint_states, params));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if(GPM_JLA_MID == inv_diff_kin_solver_params.constraint)
    {
        typedef JointLimitAvoidanceMid<ConstraintParamsJLA, PRIO> tJlaMid;
        // same params as for normal JLA
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(inv_diff_kin_solver_params, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<tJlaMid > jla(new tJlaMid(100, joint_states, params));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if(GPM_CA == inv_diff_kin_solver_params.constraint ||
            TASK_STACK == inv_diff_kin_solver_params.constraint ||
            TASK_STACK_2ND == inv_diff_kin_solver_params.constraint ||
            TASK_PRIO == inv_diff_kin_solver_params.constraint)
    {
        typedef CollisionAvoidance<ConstraintParamsCA, PRIO> tCollisionAvoidance;
        uint32_t available_dists = data_mediator.obstacleDistancesCnt();
        uint32_t startPrio = 100;
        for (uint32_t i = 0; i < available_dists; ++i)
        {
            ConstraintParamsCA params = ConstraintParamFactory<ConstraintParamsCA>::createConstraintParams(inv_diff_kin_solver_params, data_mediator);
            // TODO: take care PRIO could be of different type than UINT32
            boost::shared_ptr<tCollisionAvoidance > ca(new tCollisionAvoidance(startPrio--, joint_states, params, jnt_to_jac));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(ca));
        }
    }
    else
    {
        // Nothing to do here!
        // Create constraints will be called also in case of an unconstraint solver etc.
        // So the log would be filled unnecessarily.
    }

    return constraints;
}
/* END ConstraintsBuilder *******************************************************************************************/

/* BEGIN CollisionAvoidance *************************************************************************************/
/// Returns values of the cost function
template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getValue() const
{
    return minimal_distance_;

//    ObstacleDistanceInfo d = this->constraint_params_.current_distance_;
//    if (this->getActivationThreshold() > d.min_distance)
//        return pow(d.min_distance - this->getActivationThreshold(), 2.0);
//    else
//        return 0.0;
}

/// Returns values of the derivative of the cost function
template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getDerivativeValue() const
{
    return 0.0;
}

/// Returns the threshold of the cost function to become active.
template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getActivationThreshold() const
{
    return 0.1; // in [m]
}


template <typename T_PARAMS, typename PRIO>
Eigen::MatrixXd CollisionAvoidance<T_PARAMS, PRIO>::getObstacleAvoidancePointJac() const
{
    return obstacle_avoidance_point_jac_;
}

template <typename T_PARAMS, typename PRIO>
Eigen::Vector3d CollisionAvoidance<T_PARAMS, PRIO>::getDistanceVector() const
{
    return obstacle_avoidance_distance_vec_;
}

/// Returns values of the gradient of the cost function
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd  CollisionAvoidance<T_PARAMS, PRIO>::getPartialValues()
{
    uint8_t vecRows = static_cast<uint8_t>(this->joint_states_.current_q_.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(vecRows);
    const InvDiffKinSolverParams& params = this->constraint_params_.getInvDiffKinSolverParams();
    int size_of_frames = params.frame_names.size();
    ObstacleDistanceInfo d = this->constraint_params_.current_distance_;

    ROS_INFO_STREAM("d.min_distance: " << d.min_distance);

    if (this->getActivationThreshold() > d.min_distance)
    {
        std::vector<std::string>::const_iterator str_it = std::find(params.frame_names.begin(),
                                                                    params.frame_names.end(),
                                                                    d.frame_id);
        if (params.frame_names.end() != str_it)
        {
            Eigen::Matrix3d skew_symm;
            skew_symm <<    0.0,                          d.collision_pnt_vector.z(), -d.collision_pnt_vector.y(),
                            -d.collision_pnt_vector.z(),  0.0,                         d.collision_pnt_vector.x(),
                             d.collision_pnt_vector.y(), -d.collision_pnt_vector.x(),  0.0;

            Eigen::Matrix3d ident = Eigen::Matrix3d::Identity();
            Eigen::Matrix<double,6,6> T;
            T.block(0, 0, 3, 3) << ident;
            T.block(0, 3, 3, 3) << skew_symm;
            T.block(3, 0, 3, 3) << Eigen::Matrix3d::Zero();
            T.block(3, 3, 3, 3) << ident;


            uint32_t idx = str_it - params.frame_names.begin();
            uint32_t frame_number = idx + 1; // segment nr not index represents frame number

            KDL::Jacobian new_jac_chain(size_of_frames);

            KDL::JntArray ja = this->joint_states_.current_q_;
            this->jnt_to_jac_.JntToJac(ja, new_jac_chain, frame_number);

            //obstacle_avoidance_point_jac_ = new_jac_chain.data;
            minimal_distance_ = d.min_distance;


            t_Matrix6Xd crit_pnt_jac = T * new_jac_chain.data;


            ROS_INFO_STREAM("Transformation between segment jacobian and crit jacobian: " << std::endl << T);
            ROS_INFO_STREAM("Error between Jacobians: " << std::endl << (crit_pnt_jac - new_jac_chain.data));




            Eigen::Matrix3Xd m_transl = Eigen::Matrix3Xd::Zero(3, size_of_frames);
//            m_transl << new_jac_chain.data.row(0),
//                 new_jac_chain.data.row(1),
//                 new_jac_chain.data.row(2);

            m_transl << crit_pnt_jac.row(0),
                        crit_pnt_jac.row(1),
                        crit_pnt_jac.row(2);

            obstacle_avoidance_point_jac_ = m_transl;

//            Eigen::Matrix3Xd m_rot = Eigen::Matrix3Xd::Zero(3, size_of_frames);
//            m_rot << new_jac_chain.data.row(3),
//                 new_jac_chain.data.row(4),
//                 new_jac_chain.data.row(5);

            /* According to Schwienbacher M., Buschmann T., et al,
               "Self-Collision Avoidance and Angular Momentum Compensation for a Biped Humanoid Robot", 2011,
               IEEE International Conference on Robotics and Automation
               The calculation of the general translational Jacobian for a point p is given by (using the translational and rotational
               part of the Jacobian), else the min criteria is not sufficient for all manipulator postures:
            */
            Eigen::Vector3d vec;
            vec << d.distance_vec[0], d.distance_vec[1], d.distance_vec[2];
            obstacle_avoidance_distance_vec_ = vec;
            Eigen::VectorXd term_2nd = (m_transl.transpose()) * (vec / vec.norm()); // use the unit vector only for direction!
            //Eigen::VectorXd term_2nd = (new_jac_chain.data.transpose()) * d.distance_vec;
            ROS_INFO_STREAM(">>>>>>>>>> FULL d.distance_vec: " << std::endl << d.distance_vec.transpose());

            // Gradient of the cost function from: Strasse O., Escande A., Mansard N. et al.
            // "Real-Time (Self)-Collision Avoidance Task on a HRP-2 Humanoid Robot", 2008 IEEE International Conference
            partial_values =  (2.0 * ((d.min_distance - this->getActivationThreshold()) / d.min_distance) * term_2nd);
        }
        else
        {
            ROS_ERROR_STREAM("Frame id not found: " << d.frame_id);
        }
    }

    return partial_values;
}

/// Returns a value for k_H to weight the partial values for GPM e.g.
template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    const InvDiffKinSolverParams &params = this->constraint_params_.getInvDiffKinSolverParams();
    return SelfMotionMagnitudeFactory<SmmDeterminatorConstant >::calculate(params, particular_solution, homogeneous_solution);
}
/* END CollisionAvoidance ***************************************************************************************/

/* BEGIN JointLimitAvoidance ************************************************************************************/

template <typename T_PARAMS, typename PRIO>
Eigen::MatrixXd JointLimitAvoidance<T_PARAMS, PRIO>::getObstacleAvoidancePointJac() const
{
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(6, 7); // TODO: Correct rows and cols
    return x;
}

template <typename T_PARAMS, typename PRIO>
Eigen::Vector3d JointLimitAvoidance<T_PARAMS, PRIO>::getDistanceVector() const
{
    return Eigen::Vector3d::Zero();
}

/// Returns values of the JLA cost function with adapted joint pos.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getValue(Eigen::VectorXd steps) const
{
    const InvDiffKinSolverParams &params = this->constraint_params_.getInvDiffKinSolverParams();
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;
    const KDL::JntArray joint_pos = this->joint_states_.current_q_;
    double H_q = 0.0;
    for(uint8_t i = 0; i < joint_pos.rows() ; ++i)
    {
        double jnt_pos_with_step = joint_pos(i) + steps(i);
        double nom = pow(limits_max[i] - limits_min[i], 2.0);
        double denom = (limits_max[i] - jnt_pos_with_step) * (jnt_pos_with_step - limits_min[i]);
        H_q += nom / denom;
    }

    H_q = H_q / 4.0;
    return H_q;
}

/// Returns values of the JLA cost function.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getValue() const
{
    return this->getValue(Eigen::VectorXd::Zero(this->joint_states_.current_q_.rows()));
}

/// Returns values of the derivative of the JLA cost function
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getDerivativeValue() const
{
    return 0.0;
}

/// Returns the threshold of the cost function to become active.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getActivationThreshold() const
{
    return 0.0;
}

/// Returns a value for k_H to weight the partial values for GPM e.g.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    // k_H by Armijo-Rule
    double t;
    const InvDiffKinSolverParams &params = this->constraint_params_.getInvDiffKinSolverParams();
    t = SelfMotionMagnitudeFactory< SmmDeterminatorVelocityBounds<MIN_CRIT> >::calculate(params, particular_solution, homogeneous_solution);
    return t;
}

/// Returns values of the gradient of the cost function
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidance<T_PARAMS, PRIO>::getPartialValues()
{
    const InvDiffKinSolverParams &params = this->constraint_params_.getInvDiffKinSolverParams();
    const KDL::JntArray joint_pos = this->joint_states_.current_q_;
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;
    uint8_t vec_rows = static_cast<uint8_t>(joint_pos.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(vec_rows);
    for(uint8_t i = 0; i < joint_pos.rows() ; ++i)
    {
        partial_values(i) = 0.0; // in the else cases -> output always 0
        //See Chan paper ISSN 1042-296X [Page 288]
        double min_delta = (joint_pos(i) - limits_min[i]);
        double max_delta = (limits_max[i] - joint_pos(i));
        double nominator = (2.0 * joint_pos(i) - limits_min[i] - limits_max[i]) * (limits_max[i] - limits_min[i]) * (limits_max[i] - limits_min[i]);
        double denom = 4.0 * min_delta * min_delta * max_delta * max_delta;
        partial_values(i) = nominator / denom;
    }

    return partial_values;
}
/* END JointLimitAvoidance **************************************************************************************/

/* BEGIN 2nd JointLimitAvoidance ************************************************************************************/

template <typename T_PARAMS, typename PRIO>
Eigen::Vector3d JointLimitAvoidanceMid<T_PARAMS, PRIO>::getDistanceVector() const
{
    return Eigen::Vector3d::Zero();
}


template <typename T_PARAMS, typename PRIO>
Eigen::MatrixXd JointLimitAvoidanceMid<T_PARAMS, PRIO>::getObstacleAvoidancePointJac() const
{
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(6, 7); // TODO: Correct rows and cols
    return x;
}

/// Returns values of the JLA_Mid cost function.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getValue() const
{
    return 0.0;
}

/// Returns values of the derivative of the JLA cost function
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getDerivativeValue() const
{
    return 0.0;
}

/// Returns the threshold of the cost function to become active.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getActivationThreshold() const
{
    return 0.0;
}

/// Returns values of the gradient of the cost function. Method proposed by Liegeois
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidanceMid<T_PARAMS, PRIO>::getPartialValues()
{
    const InvDiffKinSolverParams &params = this->constraint_params_.getInvDiffKinSolverParams();
    const KDL::JntArray joint_pos = this->joint_states_.current_q_;
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;

    double rad = M_PI / 180.0;
    uint8_t vec_rows = static_cast<uint8_t>(joint_pos.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(vec_rows);

    for(uint8_t i = 0; i < vec_rows; ++i)
    {
        double min_delta = joint_pos(i) - limits_min[i];
        double max_delta = limits_max[i] - joint_pos(i);
        if( min_delta * max_delta < 0.0)
        {
            ROS_WARN_STREAM("Limit of joint " << int(i) << " reached: " << std::endl
                            << "pos=" << joint_pos(i) << ";lim_min=" << limits_min[i] << ";lim_max=" << limits_max[i]);
        }

        //Liegeois method can also be found in Chan paper ISSN 1042-296X [Page 288]
        double limits_mid = 1.0 / 2.0 * (limits_max[i] + limits_min[i]);
        double nominator = joint_pos(i) - limits_mid;
        double denom = limits_max[i] - limits_min[i];
        partial_values(i) = nominator / denom;
    }

    return partial_values;
}

/// Returns a value for k_H to weight the partial values for GPM e.g.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
{
    const InvDiffKinSolverParams &params = this->constraint_params_.getInvDiffKinSolverParams();
    return SelfMotionMagnitudeFactory<SmmDeterminatorVelocityBounds<MAX_CRIT> >::calculate(params, particular_solution, homogeneous_solution);
}
/* END 2nd JointLimitAvoidance **************************************************************************************/

#endif /* CONSTRAINT_IMPL_H_ */
