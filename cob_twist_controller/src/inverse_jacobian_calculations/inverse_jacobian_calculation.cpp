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
    Eigen::MatrixXd pseudoInverseJacobian;

    singularValuesInv=db->calc_damped_singulars(svd.singularValues());

    pseudoInverseJacobian = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();

    return pseudoInverseJacobian;
}

