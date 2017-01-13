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
 *   Authors: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *   Bruno Brito, email: Bruno.Brito@ipa.fraunhofer.de
 *
 * \date Date of creation: March, 2015
 *
 * \brief
 *   This module contains the implementation of all available
 *   damping methods.
 *
 ****************************************************************/
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
 * Method just returns a null damping factor matrix from ros parameter server.
 */
inline Eigen::MatrixXd DampingNone::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                            const Eigen::MatrixXd& jacobian_data) const
{
    Eigen::MatrixXd zero_matrix = Eigen::MatrixXd::Zero(rows,rows);;
    return zero_matrix.setZero();
}
/* END DampingNone **********************************************************************************************/


/* BEGIN DampingConstant ****************************************************************************************/
/**
 * Method just returns the damping factor.
 */
inline Eigen::MatrixXd DampingConstant::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                const Eigen::MatrixXd& jacobian_data) const
{
    uint32_t rows = sorted_singular_values.rows();
    Eigen::MatrixXd damping_factor = Eigen::MatrixXd::Identity(rows,rows) * pow(this->params_.damping_factor,2);
    return damping_factor;
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
    uint32_t rows = sorted_singular_values.rows();
    Eigen::MatrixXd prod = jacobian_data * jacobian_data.transpose();
    double d = prod.determinant();
    double w = std::sqrt(std::abs(d));
    Eigen::MatrixXd damping_factor = Eigen::MatrixXd::Identity(rows,rows);

    if (w < w_threshold)
    {
        double tmp_w = (1 - w / w_threshold);
        damping_factor = Eigen::MatrixXd::Identity(rows,rows) * pow(lambda_max * tmp_w * tmp_w,2);
    }
    else
    {
        damping_factor.setZero();
    }

    return damping_factor;
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
    Eigen::MatrixXd lambda = Eigen::MatrixXd::Identity(rows,rows);
    if (least_singular_value < this->params_.eps_damping)
    {
        double lambda_quad = pow(this->params_.lambda_max, 2.0);
        //The value was powered squared later so the sqrt was removed
        lambda = Eigen::MatrixXd::Identity(rows,rows)* (1.0 - pow(least_singular_value / this->params_.eps_damping, 2.0)) * lambda_quad;

        return lambda;
    }
    else
    {
        return lambda.setZero();
    }
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
    Eigen::MatrixXd damping_factor = Eigen::MatrixXd::Zero(rows,rows);
    for(unsigned i = 0; i < sorted_singular_values.rows(); i++)
    {
      if (sorted_singular_values[i] < this->params_.eps_damping)
      {
        double lambda_sig = params_.lambda_max /( 1+ exp((sorted_singular_values[i] + params_.w_threshold) / params_.slope_damping));
        damping_factor(i,i)=lambda_sig;
      }

    }

    return damping_factor;
}
/* END DampingSigmoid ************************************************************************************/
