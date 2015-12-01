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
        default:
            ROS_ERROR("DampingMethod %d not defined! Aborting!", params.damping_method);
            break;
    }

    return db;
}
/* END DampingBuilder *******************************************************************************************/


/* BEGIN DampingNone ********************************************************************************************/
/**
 * Method just returns the damping factor from ros parameter server.
 */
inline double DampingNone::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                            const Eigen::MatrixXd& jacobian_data) const
{
    return 0.0;
}
/* END DampingNone **********************************************************************************************/


/* BEGIN DampingConstant ****************************************************************************************/
/**
 * Method just returns the damping factor from ros parameter server.
 */
inline double DampingConstant::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                const Eigen::MatrixXd& jacobian_data) const
{
    return this->params_.damping_factor;
}
/* END DampingConstant ******************************************************************************************/


/* BEGIN DampingManipulability **********************************************************************************/
/**
 * Method returns the damping factor according to the manipulability measure.
 * [Nakamura, "Advanced Robotics Redundancy and Optimization", ISBN: 0-201-15198-7, Page 268]
 */
double DampingManipulability::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                               const Eigen::MatrixXd& jacobian_data) const
{
    double w_threshold = this->params_.w_threshold;
    double lambda_max = this->params_.lambda_max;
    Eigen::MatrixXd prod = jacobian_data * jacobian_data.transpose();
    double d = prod.determinant();
    double w = std::sqrt(std::abs(d));
    double damping_factor;

    if (w < w_threshold)
    {
        double tmp_w = (1 - w / w_threshold);
        damping_factor = lambda_max * tmp_w * tmp_w;
    }
    else
    {
        damping_factor = 0.0;
    }

    return damping_factor;
}
/* END DampingManipulability ************************************************************************************/


/* BEGIN DampingLeastSingularValues **********************************************************************************/
/**
 * Method returns the damping factor according to the least singular value.
 */
double DampingLeastSingularValues::getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                    const Eigen::MatrixXd& jacobian_data) const
{
    // Formula 15 Singularity-robust Task-priority Redundandancy Resolution
    double least_singular_value = sorted_singular_values(sorted_singular_values.rows() - 1);
    if (least_singular_value < this->params_.eps_damping)
    {
        double lambda_quad = pow(this->params_.lambda_max, 2.0);
        return sqrt( (1.0 - pow(least_singular_value / this->params_.eps_damping, 2.0)) * lambda_quad);
    }
    else
    {
        return 0.0;
    }
}
/* END DampingLeastSingularValues ************************************************************************************/

