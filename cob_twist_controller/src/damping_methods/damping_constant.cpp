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
 *   Implementation of a constant damping method
 *
 ****************************************************************/
#include "cob_twist_controller/damping_methods/damping_constant.h"

DampingConstant::DampingConstant(AugmentedSolverParams &asSolverParams, Matrix6Xd &jacobianData)
    : DampingBase(asSolverParams, jacobianData)
{

}

double DampingConstant::get_damping_factor() const
{
    return this->asSolverParams_.damping_factor;
}
