/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2017 \n
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
 *   Author: Bruno Brito, email: Bruno.Brito@ipa.fraunhofer.de
 *
 * \date Date of creation: March, 2017
 *
 * \brief
 *   This header contains the interface description of constraint solvers
 *   Pure virtual methods have to be implemented in subclasses
 *
 ****************************************************************/

#include "cob_twist_controller/constraint_solvers/solvers/unified_joint_limit_singularity_solver.h"

/**
 * Specific implementation simultaneous singularity and joint limit avoidance solver.
 * This work is based in the general weighted least norm and unified weighted least norm methods.
 */
Eigen::MatrixXd UnifiedJointLimitSingularitySolver::solve(const Vector6d_t& in_cart_velocities,
                                               const JointStates& joint_states)
{

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->jacobian_data_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd J=this->jacobian_data_;
    Eigen::MatrixXd J_T=J.transpose();
    Eigen::MatrixXd U=svd.matrixU();
    Eigen::MatrixXd U_T =U.transpose();

    Eigen::MatrixXd Q = this->calculateWeighting(in_cart_velocities, joint_states);

    Eigen::MatrixXd Wt=Q*svd.matrixV();
    Eigen::MatrixXd Wt_T=Wt.transpose();
/*
    Eigen::VectorXd singularValues = svd.singularValues();
    for (int i=0;i<singularValues.rows();i++){
        singularValues(i)=singularValues(i)*Q(i,i);
    }
    Eigen::MatrixXd lambda = damping_->getDampingFactor(singularValues, this->jacobian_data_);

    Eigen::MatrixXd Ulambda =U*lambda;
    Eigen::MatrixXd UlambdaUt =Ulambda*U_T;

    Eigen::MatrixXd WtWt_T= Wt*Wt_T;
    Eigen::MatrixXd JWtWt_T=J*Wt*Wt_T;
    Eigen::MatrixXd JWtWt_TJ_T=JWtWt_T*J_T;

    Eigen::MatrixXd J_robust=JWtWt_TJ_T+UlambdaUt;


    Eigen::MatrixXd pinv=J_robust.inverse();
    Eigen::MatrixXd J_Tpinv= J_T*pinv;
    Eigen::MatrixXd WtWt_TJ_Tpinv= WtWt_T*J_Tpinv;
    Eigen::MatrixXd qdots_out =WtWt_TJ_Tpinv*in_cart_velocities;*/

    Eigen::MatrixXd J_robust=J*Wt*Wt_T*J_T;
    Eigen::MatrixXd pinv=pinv_calc_.calculate(this->params_, this->damping_, J_robust);
    Eigen::MatrixXd qdots_out =Wt*Wt_T*J_T*pinv*in_cart_velocities;

    return qdots_out;
}

/**
 * This function returns the identity as weighting matrix for base functionality.
 */
Eigen::MatrixXd UnifiedJointLimitSingularitySolver::calculateWeighting(const Vector6d_t& in_cart_velocities, const JointStates& joint_states) const
{
    Eigen::MatrixXd Jinv = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
    Eigen::VectorXd q_dot = Jinv * in_cart_velocities;
    std::vector<double> limits_min = this->limiter_params_.limits_min;
    std::vector<double> limits_max = this->limiter_params_.limits_max;
    uint32_t cols = this->jacobian_data_.cols();

    Eigen::VectorXd weighting = Eigen::VectorXd::Zero(cols);
    Eigen::VectorXd weighting_pos = Eigen::VectorXd::Zero(cols);
    Eigen::VectorXd weighting_speed = Eigen::VectorXd::Zero(cols);

    KDL::JntArray q = joint_states.current_q_;

    double sigma = this->params_.damping_jla;
    double sigma_speed = this->params_.damping_speed_jla;
    double delta_pos = this->params_.thresholds_jla.activation_position_threshold_jla;
    double delta_speed = this->params_.thresholds_jla.activation_speed_threshold_jla;
    for (uint32_t i = 0; i < cols ; ++i)
    {

        weighting(i) = (1.0/(1.0+exp(-(q(i)-limits_min[i]-delta_pos)/sigma)))*(1.0/(1.0+exp((q(i)-limits_max[i]+delta_pos)/sigma)))+(1.0/(1.0+exp((q(i)*q_dot(i)+delta_speed)*sigma_speed)));

        if( (fabs(q(i)-limits_min[i])<params_.limiter_params.limits_tolerance*0.01745329251)){
            ROS_WARN("Joint %i tolerance distance to minimum position %f not respected",i,(fabs(q(i)-limits_min[i])));
        }
        if( (fabs(q(i)-limits_max[i])<params_.limiter_params.limits_tolerance*0.01745329251) ){
            ROS_WARN("Joint %i tolerance distance to maximum position %f not respected",i,(fabs(q(i)-limits_max[i])));
        }
        if( (fabs(q(i)-limits_min[i])<0.0)){
            ROS_ERROR("Joint %i distance to minimum position %f not respected",i,(fabs(q(i)-limits_min[i])));
        }
        if( (fabs(q(i)-limits_max[i])<0.0)){
            ROS_ERROR("Joint %i distance to maximum position %f not respected",i,(fabs(q(i)-limits_max[i])));
        }

        if(weighting(i)>1.0)
                    weighting(i)=1.0;
    }

    return weighting.asDiagonal();
}
