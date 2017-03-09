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
    /*
	Eigen::MatrixXd W_WLN = this->calculateWeighting(in_cart_velocities, joint_states);
    // for the following formulas see Chan paper ISSN 1042-296X [Page 288]
    Eigen::MatrixXd root_W_WLN =  W_WLN.cwiseSqrt();            // element-wise sqrt -> ok because diag matrix W^(1/2)
    Eigen::MatrixXd inv_root_W_WLN =  root_W_WLN.inverse();     // -> W^(-1/2)

    // SVD of JLA weighted Jacobian: Damping will be done later in calculatePinvJacobianBySVD for pseudo-inverse Jacobian with additional truncation etc.
    Eigen::MatrixXd weighted_jacobian = this->jacobian_data_ * inv_root_W_WLN;
    Eigen::MatrixXd pinv = pinv_calc_.calculate(this->params_, this->damping_, weighted_jacobian);

    ROS_INFO_STREAM("wEIGTH:"<<W_WLN(1,1));
    Eigen::MatrixXd qdots_out2 = pinv * in_cart_velocities;
    ROS_INFO_STREAM("Joint Velocity no JLA:"<<qdots_out2(1));


    // Take care: W^(1/2) * q_dot = weighted_pinv_J * x_dot -> One must consider the weighting!!!

    Eigen::MatrixXd qdots_out = inv_root_W_WLN * pinv * in_cart_velocities;
    ROS_INFO_STREAM("Joint Velocity:"<<qdots_out(1));
    return qdots_out;
*/
	//Method simplified based on General Weighted LeastNorm Control for Redudant Manipulators
    /*
	Eigen::MatrixXd W = this->calculateWeighting(in_cart_velocities, joint_states);
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->jacobian_data_, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXd singularValues = svd.singularValues();
	Eigen::MatrixXd W_V=W*svd.matrixV();
	Eigen::MatrixXd S=singularValues.asDiagonal();
	Eigen::MatrixXd J_W=svd.matrixU()*S*W_V.transpose();

	Eigen::MatrixXd pinv = pinv_calc_.calculate(this->params_, this->damping_, J_W);

	Eigen::VectorXd singularValues = svd.singularValues();
	Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(singularValues.rows());
	Eigen::MatrixXd lambda = damping_->getDampingFactor(singularValues, this->jacobian_data_);
	double eps_truncation = this->params_.eps_truncation;
    // small change to ref: here quadratic damping due to Control of Redundant Robot Manipulators : R.V. Patel, 2005, Springer [Page 13-14]
    for (uint32_t i = 0; i < singularValues.rows(); ++i)
    {
        double denominator = (singularValues(i) * singularValues(i) + lambda(i, i) +W_WLN(i, i));
        // singularValuesInv(i) = (denominator < eps_truncation) ? 0.0 : singularValues(i) / denominator;
        singularValuesInv(i) = singularValues(i)/ denominator;
    }

    Eigen::MatrixXd result = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();

	Eigen::MatrixXd pinv2 = pinv_calc_.calculate(this->params_, this->damping_, this->jacobian_data_);
	Eigen::MatrixXd qdots_out2 = pinv2 * in_cart_velocities;
	ROS_INFO_STREAM("Joint Velocity no JLA:"<<qdots_out2(1));
	Eigen::MatrixXd weighted_jacobian = W_WLN * this->jacobian_data_.transpose();
	Eigen::MatrixXd J_W_WLN_Jt= this->jacobian_data_* weighted_jacobian;
	Eigen::MatrixXd pinv = pinv_calc_.calculate(this->params_, this->damping_, J_W_WLN_Jt);

	Eigen::MatrixXd qdots_out =  weighted_jacobian * pinv * in_cart_velocities;
*//* simulations
    Eigen::MatrixXd weighted_jacobian = W* this->jacobian_data_.transpose();
    Eigen::MatrixXd lambda = damping_->getDampingFactor(singularValues, this->jacobian_data_);
    Eigen::MatrixXd U= svd.matrixU();
    Eigen::MatrixXd J_W_WLN_Jt= this->jacobian_data_* weighted_jacobian+svd.matrixU()*lambda*U.transpose();

    Eigen::MatrixXd pinv2 = pinv_calc_.calculate(J_W_WLN_Jt);
    Eigen::MatrixXd pinv3= weighted_jacobian*pinv2;
	Eigen::MatrixXd qdots_out =  pinv3 * in_cart_velocities;
    Eigen::MatrixXd weighted_jacobian = W* this->jacobian_data_.transpose();
    Eigen::MatrixXd lambda = damping_->getDampingFactor(singularValues, this->jacobian_data_);
    Eigen::MatrixXd J_W_WLN_Jt= this->jacobian_data_* weighted_jacobian;

    Eigen::MatrixXd pinv2 = pinv_calc_.calculate(this->params_, this->damping_, J_W_WLN_Jt);
    Eigen::MatrixXd pinv3= W*this->jacobian_data_.transpose()*pinv2;
    Eigen::MatrixXd qdots_out =  pinv3 * in_cart_velocities;*/

    //Final test
 /*   Eigen::MatrixXd W = this->calculateWeighting(in_cart_velocities, joint_states);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->jacobian_data_, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::MatrixXd Q=svd.matrixV()*W;
    Eigen::MatrixXd Qt=Q.transpose();
    Eigen::MatrixXd U=svd.matrixU();
    Eigen::MatrixXd Ut =U.transpose();
    Eigen::MatrixXd Jt = this->jacobian_data_.transpose();
    Eigen::MatrixXd S= Eigen::MatrixXd::Zero(6,7);
    Eigen::MatrixXd Sinv= Eigen::MatrixXd::Zero(7,6);
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::MatrixXd lambda = damping_->getDampingFactor(singularValues, this->jacobian_data_);
    for (int i=0;i<6;i++){
        S(i,i)=singularValues(i)*singularValues(i)*W(i,i)*W(i,i)+lambda(i,i)*lambda(i,i);

        Sinv(i,i)=singularValues(i)/S(i,i);
    }
    Eigen::MatrixXd J_W_Wt_Jt=this->jacobian_data_*Q*Qt*Jt;
    Eigen::MatrixXd J_lambda_Jt=J_W_Wt_Jt+U*lambda*lambda*Ut;
    Eigen::MatrixXd pinv = J_lambda_Jt.inverse();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd2(pinv, Eigen::ComputeThinU | Eigen::ComputeThinV);
    ROS_INFO_STREAM("w not respected"<<W);
    Eigen::MatrixXd qdots_out =  svd.matrixV() * Sinv * svd.matrixU().transpose() * in_cart_velocities;
    ROS_INFO_STREAM("q dotd"<<Sinv);
    */
    //Another final test
    //    Wt=Q*V;
    //qd=Wt*Wt'*J'*inv(J*Wt*Wt'*J'+U*alfa*U')*e_T;

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
