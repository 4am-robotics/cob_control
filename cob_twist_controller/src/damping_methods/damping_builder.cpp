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
 *   Static builder method to create damping methods dependent on parameterization.
 *
 ****************************************************************/

#include "ros/ros.h"
#include "cob_twist_controller/damping_methods/damping_builder.h"
#include "cob_twist_controller/damping_methods/damping_manipulability.h"
#include "cob_twist_controller/damping_methods/damping_constant.h"

DampingBase* DampingBuilder::create_damping(AugmentedSolverParams &augmentedSolverParams, Matrix6Xd &jacobianData)
{
    DampingBase *db = NULL;
    switch(augmentedSolverParams.damping_method)
    {
        case MANIPULABILITY:
            db = new DampingManipulability(augmentedSolverParams, jacobianData);
            break;
        case CONSTANT:
            db = new DampingConstant(augmentedSolverParams, jacobianData);
            break;
        default:
            ROS_ERROR("DampingMethod %d not defined! Aborting!", augmentedSolverParams.damping_method);
            break;
    }

    return db;
}

