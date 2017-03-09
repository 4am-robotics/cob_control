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
 *   Implementation of an JLA solver.
 *   Special constraint: Avoid joint limits.
 *
 ****************************************************************/

#include "cob_twist_controller/constraint_solvers/solvers/weighted_least_norm_solver.h"

/**
 * Specific implementation of the solve method using a weighted least norm.
 * This is done by calculation of a weighting which is dependent on inherited classes for the Jacobian.
 * Uses the base implementation of calculatePinvJacobianBySVD to calculate the pseudo-inverse (weighted) Jacobian.
 */
Eigen::MatrixXd WeightedLeastNormSolver::solve(const Vector6d_t& in_cart_velocities,
                                               const JointStates& joint_states)
{

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->jacobian_data_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd J=this->jacobian_data_;
    Eigen::MatrixXd J_T=J.transpose();
    Eigen::MatrixXd U=svd.matrixU();
    Eigen::MatrixXd U_T =U.transpose();

    Eigen::MatrixXd Q = this->calculateWeighting(in_cart_velocities, joint_states);
    Eigen::VectorXd singularValues = svd.singularValues();
    for (int i=0;i<singularValues.rows();i++){
        singularValues(i)=singularValues(i)*Q(i,i);
    }
    Eigen::MatrixXd lambda = damping_->getDampingFactor(singularValues, this->jacobian_data_);

    Eigen::MatrixXd Ulambda =U*lambda;
    Eigen::MatrixXd UlambdaUt =Ulambda*U_T;

    Eigen::MatrixXd Wt=Q*svd.matrixV();
    Eigen::MatrixXd Wt_T=Wt.transpose();


    Eigen::MatrixXd WtWt_T= Wt*Wt_T;
    Eigen::MatrixXd JWtWt_T=J*Wt*Wt_T;
    Eigen::MatrixXd JWtWt_TJ_T=JWtWt_T*J_T;

    Eigen::MatrixXd J_robust=JWtWt_TJ_T+UlambdaUt;


    Eigen::MatrixXd pinv=J_robust.inverse();
    Eigen::MatrixXd J_Tpinv= J_T*pinv;
    Eigen::MatrixXd WtWt_TJ_Tpinv= WtWt_T*J_Tpinv;
    //Eigen::MatrixXd J_robust=J*Wt*Wt_T*J_T;
    //Eigen::MatrixXd pinv=pinv_calc_.calculate(this->params_, this->damping_, J_robust);
    //Eigen::MatrixXd qdots_out =Wt*Wt_T*J_T*pinv*in_cart_velocities;

    Eigen::MatrixXd qdots_out =WtWt_TJ_Tpinv*in_cart_velocities;
	return qdots_out;
}

/**
 * This function returns the identity as weighting matrix for base functionality.
 */
Eigen::MatrixXd WeightedLeastNormSolver::calculateWeighting(const Vector6d_t& in_cart_velocities, const JointStates& joint_states) const
{
    uint32_t cols = this->jacobian_data_.cols();
    Eigen::VectorXd weighting = Eigen::VectorXd::Ones(cols);
    return weighting.asDiagonal();
}
