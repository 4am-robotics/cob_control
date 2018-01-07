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
    Eigen::MatrixXd lambda = db->getDampingFactor(singularValues, jacobian);

    if (params.numerical_filtering)
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
        singularValuesInv(m) = singularValues(m) / (pow(singularValues(m), 2) + pow(params.beta, 2) + lambda(m, m));
    }
    else
    {
        // small change to ref: here quadratic damping due to Control of Redundant Robot Manipulators : R.V. Patel, 2005, Springer [Page 13-14]
        for (uint32_t i = 0; i < singularValues.rows(); ++i)
        {
            double denominator = (singularValues(i) * singularValues(i) + lambda(i, i) );
            // singularValuesInv(i) = (denominator < eps_truncation) ? 0.0 : singularValues(i) / denominator;
            singularValuesInv(i) = (singularValues(i) < eps_truncation) ? 0.0 : singularValues(i) / denominator;
        }

        //// Formula from Advanced Robotics : Redundancy and Optimization : Nakamura, Yoshihiko, 1991, Addison-Wesley Pub. Co [Page 258-260]
        // for(uint32_t i = 0; i < singularValues.rows(); ++i)
        // {
        //       // damping is disabled due to damping factor lower than a const. limit
        //       singularValues(i) = (singularValues(i) < eps_truncation) ? 0.0 : 1.0 / singularValues(i);
        // }
    }

    Eigen::MatrixXd result = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();

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

    Eigen::MatrixXd lambda = db->getDampingFactor(Eigen::VectorXd::Zero(1, 1), jacobian);
    if (cols >= rows)
    {
        Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(rows, rows);
        Eigen::MatrixXd temp = jacobian * jac_t + lambda * ident;
        result = jac_t * temp.inverse();
    }
    else
    {
        Eigen::MatrixXd ident = Eigen::MatrixXd::Identity(cols, cols);
        Eigen::MatrixXd temp = jac_t * jacobian + lambda * ident;
        result = temp.inverse() * jac_t;
    }

    return result;
}
