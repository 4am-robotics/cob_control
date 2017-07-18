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


#include "ros/ros.h"
#include "cob_twist_controller/damping_methods/damping.h"

/* BEGIN DampingBuilder *****************************************************************************************/
/**
 * Static builder method to create damping methods dependent on parameterization.
 */
DampingBase* DampingBuilder::createDamping(const TwistControllerParams& params)
{
    DampingBase* db = NULL;
    switch (params.damping_method)
    {
        case NO_DAMPING:
            db = new DampingNone(params);
            break;
        case CONSTANT:
            db = new DampingConstant(params);
            break;
        case MANIPULABILITY:
            db = new DampingManipulability(params);
            break;
        case LEAST_SINGULAR_VALUE:
            db = new DampingLeastSingularValues(params);
            break;
        case SIGMOID:
            db = new DampingSigmoid(params);
            break;
        default:
            ROS_ERROR("DampingMethod %d not defined! Aborting!", params.damping_method);
            break;
    }

    return db;
}
/* END DampingBuilder *******************************************************************************************/


/* BEGIN DampingNone ********************************************************************************************/
/**
 * Method just returns a zero matrix.
 */
inline Eigen::MatrixXd DampingNone::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                     const Eigen::MatrixXd& jacobian_data) const
{
    uint32_t rows = sorted_singular_values.rows();
    return Eigen::MatrixXd::Zero(rows, rows);
}
/* END DampingNone **********************************************************************************************/


/* BEGIN DampingConstant ****************************************************************************************/
/**
 * Method just returns the damping factor from ros parameter server.
 */
inline Eigen::MatrixXd DampingConstant::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                         const Eigen::MatrixXd& jacobian_data) const
{
    uint32_t rows = sorted_singular_values.rows();
    return Eigen::MatrixXd::Identity(rows, rows) * pow(this->params_.damping_factor, 2);
}
/* END DampingConstant ******************************************************************************************/


/* BEGIN DampingManipulability **********************************************************************************/
/**
 * Method returns the damping factor according to the manipulability measure.
 * [Nakamura, "Advanced Robotics Redundancy and Optimization", ISBN: 0-201-15198-7, Page 268]
 */
Eigen::MatrixXd DampingManipulability::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                        const Eigen::MatrixXd& jacobian_data) const
{
    double w_threshold = this->params_.w_threshold;
    double lambda_max = this->params_.lambda_max;
    Eigen::MatrixXd prod = jacobian_data * jacobian_data.transpose();
    double d = prod.determinant();
    double w = std::sqrt(std::abs(d));
    double damping_factor;
    uint32_t rows = sorted_singular_values.rows();
    Eigen::MatrixXd damping_matrix = Eigen::MatrixXd::Zero(rows, rows);

    if (w < w_threshold)
    {
        double tmp_w = (1 - w / w_threshold);
        damping_factor = lambda_max * tmp_w * tmp_w;
        damping_matrix = Eigen::MatrixXd::Identity(rows, rows) * pow(damping_factor, 2);
    }

    return damping_matrix;
}
/* END DampingManipulability ************************************************************************************/


/* BEGIN DampingLeastSingularValues **********************************************************************************/
/**
 * Method returns the damping factor according to the least singular value.
 */
Eigen::MatrixXd DampingLeastSingularValues::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                             const Eigen::MatrixXd& jacobian_data) const
{
    // Formula 15 Singularity-robust Task-priority Redundandancy Resolution
    double least_singular_value = sorted_singular_values(sorted_singular_values.rows() - 1);
    uint32_t rows = sorted_singular_values.rows();
    Eigen::MatrixXd damping_matrix = Eigen::MatrixXd::Zero(rows, rows);

    if (least_singular_value < this->params_.eps_damping)
    {
        double lambda_quad = pow(this->params_.lambda_max, 2.0);
        double damping_factor = sqrt( (1.0 - pow(least_singular_value / this->params_.eps_damping, 2.0)) * lambda_quad);
        damping_matrix = Eigen::MatrixXd::Identity(rows, rows) * pow(damping_factor, 2);
    }

    return damping_matrix;
}
/* END DampingLeastSingularValues ************************************************************************************/

/* BEGIN DampingSigmoid **********************************************************************************/
/**
 * Method returns the damping factor based on a sigmoid function on the value of each singular value.
 */
Eigen::MatrixXd DampingSigmoid::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                 const Eigen::MatrixXd& jacobian_data) const
{
    // Formula will be described in a future paper (to add reference)
    uint32_t rows = sorted_singular_values.rows();
    Eigen::MatrixXd damping_matrix = Eigen::MatrixXd::Zero(rows, rows);

    for (unsigned i = 0; i < rows; i++)
    {

        double lambda_sig = params_.lambda_max / (1 + exp((sorted_singular_values[i] + params_.w_threshold) / params_.slope_damping));
        damping_matrix(i, i) = lambda_sig;
    }

    return damping_matrix;
}
/* END DampingSigmoid ************************************************************************************/
