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
DampingBase* DampingBuilder::create_damping(AugmentedSolverParams &augmentedSolverParams, Matrix6Xd &jacobianData)
{
    DampingBase *db = NULL;
    switch(augmentedSolverParams.damping_method)
    {
        case NONE:
            db = new DampingNone(augmentedSolverParams, jacobianData);
            break;
        case CONSTANT:
            db = new DampingConstant(augmentedSolverParams, jacobianData);
            break;
        case MANIPULABILITY:
            db = new DampingManipulability(augmentedSolverParams, jacobianData);
            break;
        case LSV:
            db = new DampingLeastSingularValues(augmentedSolverParams, jacobianData);
            break;
        default:
            ROS_ERROR("DampingMethod %d not defined! Aborting!", augmentedSolverParams.damping_method);
            break;
    }

    return db;
}
/* END DampingBuilder *******************************************************************************************/

/* BEGIN DampingNone ********************************************************************************************/
/**
 * Method just returns the damping factor from ros parameter server.
 */
inline double DampingNone::get_damping_factor(const Eigen::VectorXd &sortedSingularValues) const
{
    return 0.0;
}

/* END DampingNone **********************************************************************************************/

/* BEGIN DampingConstant ****************************************************************************************/
/**
 * Method just returns the damping factor from ros parameter server.
 */
inline double DampingConstant::get_damping_factor(const Eigen::VectorXd &sortedSingularValues) const
{
    return this->asParams_.damping_factor;
}
/* END DampingConstant ******************************************************************************************/

/* BEGIN DampingManipulability **********************************************************************************/
/**
 * Method returns the damping factor according to the manipulability measure.
 * [Nakamura, "Advanced Robotics Redundancy and Optimization", ISBN: 0-201-15198-7, Page 268]
 */
double DampingManipulability::get_damping_factor(const Eigen::VectorXd &sortedSingularValues) const
{
    double w_threshold = this->asParams_.w_threshold;
    double lambda_max = this->asParams_.lambda_max;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = this->jacobianData_ * this->jacobianData_.transpose();
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

double DampingLeastSingularValues::get_damping_factor(const Eigen::VectorXd &sortedSingularValues) const
{
    // Formula 15 Singularity-robust Task-priority Redundandancy Resolution
    if((double)sortedSingularValues(sortedSingularValues.rows()-1) < this->asParams_.eps_damping)
    {
        return sqrt( (1-pow((double)sortedSingularValues(sortedSingularValues.rows()-1)/this->asParams_.eps_damping,2)) * pow(this->asParams_.lambda_max,2) );
    }else
    {
        return 0;
    }
}
/* END DampingLeastSingularValues ************************************************************************************/

