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
#include <cob_twist_controller/pseudoinverse_calculations/pseudoinverse_calculation.h>

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

