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
using namespace Eigen;

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
        case LOWISO:
            db = new DampingLowIsotropic(augmentedSolverParams, jacobianData);
            break;
        default:
            ROS_ERROR("DampingMethod %d not defined! Aborting!", augmentedSolverParams.damping_method);
            break;
    }

    return db;
}
/* END DampingBuilder *******************************************************************************************/

/* BEGIN DampingConstant ****************************************************************************************/
/**
 * Method just returns the damping factor from ros parameter server.
 */
inline double DampingConstant::get_damping_factor() const
{
    return this->asParams_.damping_factor;
}
/* END DampingConstant ******************************************************************************************/

/* BEGIN DampingManipulability **********************************************************************************/
/**
 * Method returns the damping factor according to the manipulability measure.
 * [Nakamura, "Advanced Robotics Redundancy and Optimization", ISBN: 0-201-15198-7, Page 268]
 */
double DampingManipulability::get_damping_factor() const
{
    double wt = this->asParams_.wt;
    double lambda0 = this->asParams_.lambda0;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = this->jacobianData_ * this->jacobianData_.transpose();
    double d = prod.determinant();
    double w = std::sqrt(std::abs(d));
    double damping_factor;
    if (w < wt)
    {
        double tmp_w = (1 - w / wt);
        damping_factor = lambda0 * tmp_w * tmp_w;
    }
    else
    {
        damping_factor = 0.0;
    }

    return damping_factor;
}

/* END DampingManipulability ************************************************************************************/

/* BEGIN DampingLowIsotropic **********************************************************************************/
/**
 * Method returns the damping factor according to the manipulability measure.
 * [Nakamura, "Advanced Robotics Redundancy and Optimization", ISBN: 0-201-15198-7, Page 268]
 */
double DampingLowIsotropic::get_damping_factor() const
{
    return 0;
}

VectorXd DampingLowIsotropic::calc_damped_singulars(VectorXd sortedSingValues) const{
    double lambda;
    uint32_t i = 0;
    VectorXd singularValuesAdapted = VectorXd::Zero(sortedSingValues.rows());

    // Formula 15 Singularity-robust Task-priority Redundandancy Resolution
    if((double)sortedSingValues(sortedSingValues.rows()-1) < this->asParams_.eps){
        lambda = sqrt( (1-pow((double)sortedSingValues(sortedSingValues.rows()-1)/this->asParams_.eps,2)) * pow(this->asParams_.lambda_max,2) );
    }else{
        lambda=0;
    }

    for(; i < sortedSingValues.rows()-1; ++i)
    {
        // beta² << lambda²
        singularValuesAdapted(i) = sortedSingValues(i) / ( pow((double)sortedSingValues(i),2) + pow(this->asParams_.beta,2) );
    }
    singularValuesAdapted(i) = sortedSingValues(i) / ( pow((double)sortedSingValues(i),2) + pow(this->asParams_.beta,2) + pow(lambda,2) );
    return singularValuesAdapted;
}
/* END DampingLowIsotropic ************************************************************************************/


/* BEGIN DampingNone ********************************************************************************************/
/**
 * Method just returns the damping factor from ros parameter server.
 */
inline double DampingNone::get_damping_factor() const
{
    return 0.0;
}

VectorXd DampingNone::calc_damped_singulars(VectorXd sortedSingValues) const{
    // Formula from Advanced Robotics : Redundancy and Optimization : Nakamura, Yoshihiko, 1991, Addison-Wesley Pub. Co [Page 258-260]
    for(uint32_t i = 0; i < sortedSingValues.rows(); ++i)
    {
        // damping is disabled due to damping factor lower than a const. limit
        sortedSingValues(i) = (sortedSingValues(i) < this->asParams_.eps) ? 0.0 : 1.0 / sortedSingValues(i);
    }

    return sortedSingValues;
}
/* END DampingNone **********************************************************************************************/
