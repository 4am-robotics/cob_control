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
 *   Implementation pseudoinverse calculation classes.
 *
 ****************************************************************/
#include <cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h>
#include <Eigen/Core>
#include <Eigen/SVD>

/**
 * Calculates the pseudoinverse of the Jacobian by using SVD technique.
 * This allows to get information about singular values and evaluate them.
 */
Eigen::MatrixXd PInvBySVD::calculate(const AugmentedSolverParams& params,
                                                 boost::shared_ptr<DampingBase> db,
                                                 const Eigen::MatrixXd& jacobian) const
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double eps = params.eps;
    double lambda = db->get_damping_factor();
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(singularValues.rows());

    // Formula from Advanced Robotics : Redundancy and Optimization : Nakamura, Yoshihiko, 1991, Addison-Wesley Pub. Co [Page 258-260]
    for(uint32_t i = 0; i < singularValues.rows(); ++i)
    {
        // small change to ref: here quadratic damping due to Control of Redundant Robot Manipulators : R.V. Patel, 2005, Springer [Page 13-14]
        if (NONE == params.damping_method || DAMPING_LIMIT > lambda)
        {
            // damping is disabled due to damping factor lower than a const. limit
            singularValuesInv(i) = (singularValues(i) < eps) ? 0.0 : 1.0 / singularValues(i);
        }
        else
        {
            double denominator = (singularValues(i) * singularValues(i) + lambda * lambda);
            singularValuesInv(i) = (denominator < eps) ? 0.0 : singularValues(i) / denominator;
        }
    }

    Eigen::MatrixXd pseudoInverseJacobian = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
    return pseudoInverseJacobian;
}




Eigen::MatrixXd PInvByLID::calculate(const AugmentedSolverParams& params,
                                                 boost::shared_ptr<DampingBase> db,
                                                 const Eigen::MatrixXd& jacobian) const
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double eps = params.eps;
    double lambda;// = db->get_damping_factor();
    double lambda_max = 0.5;
    double beta = params.beta;
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(singularValues.rows());
    Eigen::MatrixXd pseudoInverseJacobian_1 = Eigen::MatrixXd::Zero(svd.matrixV().rows(),svd.matrixU().rows());

    // Formula 15 Singularity-robust Task-priority Redundandancy Resolution
    if((double)singularValues(singularValues.rows()-1) < eps){
        lambda = sqrt( (1-pow((double)singularValues(singularValues.rows()-1)/eps,2)) * pow(lambda_max,2) );
    }else{
        lambda=0;
    }

    uint32_t i = 0;
    for(; i < singularValues.rows()-1; ++i)
    {
        // beta² << lambda²
        pseudoInverseJacobian_1 += singularValues(i) / ( pow((double)singularValues(i),2) + pow(beta,2) ) * svd.matrixV().col(i) * svd.matrixU().col(i).transpose();
    }


    Eigen::MatrixXd pseudoInverseJacobian_2 = singularValues(i) / ( pow((double)singularValues(i),2) + pow(beta,2) + pow(lambda,2) ) * svd.matrixV().col(i) * svd.matrixU().col(i).transpose();


    Eigen::MatrixXd pseudoInverseJacobian = pseudoInverseJacobian_1 + pseudoInverseJacobian_2;
    return pseudoInverseJacobian;
}
