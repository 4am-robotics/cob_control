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
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/SVD>

#include <cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h>

/**
 * Calculates the pseudoinverse of the Jacobian by using SVD technique.
 * This allows to get information about singular values and evaluate them.
 */
Eigen::MatrixXd PInvBySVD::calculate(const Eigen::MatrixXd& jacobian) const
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double eps_truncation = DIV0_SAFE;  // prevent division by 0.0
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(singularValues.rows());

    // small change to ref: here quadratic damping due to Control of Redundant Robot Manipulators : R.V. Patel, 2005, Springer [Page 13-14]

    //ToDo: Test this
    for (uint32_t i = 0; i < singularValues.rows(); ++i)
    {
        double denominator = singularValues(i) * singularValues(i);
        // singularValuesInv(i) = (denominator < eps_truncation) ? 0.0 : singularValues(i) / denominator;
        singularValuesInv(i) = (singularValues(i) < eps_truncation) ? 0.0 : singularValues(i) / denominator;
    }

    Eigen::MatrixXd result = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();

    return result;
}

/**
 * Calculates the pseudoinverse of the Jacobian by using SVD technique.
 * This allows to get information about singular values and evaluate them.
 */
Eigen::MatrixXd PInvBySVD::calculate(const TwistControllerParams& params,
                                     boost::shared_ptr<DampingBase> db,
                                     const Eigen::MatrixXd& jacobian) const
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double eps_truncation = params.eps_truncation;
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::VectorXd singularValuesInv = Eigen::VectorXd::Zero(singularValues.rows());
    double lambda = db->getDampingFactor(singularValues, jacobian);
    Eigen::MatrixXd result;

    switch(params.singularity_avoidance)
    {
        case(SA_NF):
        {
            // Formula 20 Singularity-robust Task-priority Redundandancy Resolution
            // Sum part
            for (uint32_t i = 0; i < singularValues.rows()-1; ++i)
            {
                // pow(beta, 2) << pow(lambda, 2)
                singularValuesInv(i) = singularValues(i) / (pow(singularValues(i), 2) + pow(params.beta, 2));
            }
            // Formula 20 - additional part - numerical filtering for least singular value m
            uint32_t m = singularValues.rows()-1;
            singularValuesInv(m) = singularValues(m) / (pow(singularValues(m), 2) + pow(params.beta, 2) + pow(lambda, 2));

            result = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
            break;
        }
        case(SA_SR):
        {
            Eigen::VectorXd lambda_vec = Eigen::VectorXd::Zero(singularValues.size());

            Eigen::MatrixXd S = Eigen::MatrixXd::Zero(singularValues.size(),singularValues.size());

            for(unsigned i = 0; i < singularValues.size(); i++)
            {
                lambda_vec(i) = params.damping_gain /( 1+ exp(static_cast <double>(singularValues(i)) + params.damping_delta) / params.damping_slope);
                S(i,i) = singularValues(i)/(pow(static_cast <double>(singularValues(i)),2) + lambda_vec(i));
            }

            result = svd.matrixV() * S * svd.matrixU().transpose();
            break;
        }
        case(SA_QD):
        {
            for (uint32_t i = 0; i < singularValues.rows(); ++i)
            {
                double denominator = singularValues(i) * singularValues(i);
                singularValuesInv(i) = (singularValues(i) < eps_truncation) ? 0.0 : singularValues(i) / denominator;
            }

            result = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
            break;
        }
        case(SA_QD_EXTENDED):
        {
            // small change to ref: here quadratic damping due to Control of Redundant Robot Manipulators : R.V. Patel, 2005, Springer [Page 13-14]
            for (uint32_t i = 0; i < singularValues.rows(); ++i)
            {
                double denominator = (singularValues(i) * singularValues(i) + pow(lambda, 2) );
                // singularValuesInv(i) = (denominator < eps_truncation) ? 0.0 : singularValues(i) / denominator;
                singularValuesInv(i) = (singularValues(i) < eps_truncation) ? 0.0 : singularValues(i) / denominator;
            }

            result = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
        }
    }

    return result;
}

/**
 * Calculates the pseudoinverse by means of left/right pseudo inverse respectively.
 */
Eigen::MatrixXd PInvDirect::calculate(const Eigen::MatrixXd& jacobian) const
{
    Eigen::MatrixXd result;
    Eigen::MatrixXd jac_t = jacobian.transpose();
    uint32_t rows = jacobian.rows();
    uint32_t cols = jacobian.cols();

    if (cols >= rows)
    {
        result = jac_t * (jacobian * jac_t).inverse();
    }
    else
    {
        result = (jac_t * jacobian).inverse() * jac_t;
    }

    return result;
}

/**
 * Calculates the pseudoinverse by means of left/right pseudo inverse respectively.
 */
Eigen::MatrixXd PInvDirect::calculate(const TwistControllerParams& params,
                                      boost::shared_ptr<DampingBase> db,
                                      const Eigen::MatrixXd& jacobian) const
{
    Eigen::MatrixXd result;
    Eigen::MatrixXd jac_t = jacobian.transpose();
    uint32_t rows = jacobian.rows();
    uint32_t cols = jacobian.cols();
    if (params.damping_method == LEAST_SINGULAR_VALUE)
    {
        ROS_ERROR("PInvDirect does not support SVD. Use PInvBySVD class instead!");
    }

    double lambda = db->getDampingFactor(Eigen::VectorXd::Zero(1, 1), jacobian);
    if (cols >= rows)
    {
        Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(rows, rows);
        Eigen::MatrixXd temp = jacobian * jac_t + lambda * lambda * ident;
        result = jac_t * temp.inverse();
    }
    else
    {
        Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(cols, cols);
        Eigen::MatrixXd temp = jac_t * jacobian + lambda * lambda * ident;
        result = temp.inverse() * jac_t;
    }

    return result;
}
