/*
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
 *   This header contains the description of a class providing a static method to create damping method objects.
 *
 ****************************************************************/
#ifndef DAMPING_BUILDER_H_
#define DAMPING_BUILDER_H_

#include "cob_twist_controller/augmented_solver_data_types.h"
#include "damping_base.h"

class DampingBuilder
{
    public:
        static DampingBase* create_damping(AugmentedSolverParams &augmentedSolverParams, Matrix6Xd &jacobianData);

    private:
        DampingBuilder() {}
        ~DampingBuilder() {}
};

#endif /* DAMPING_BUILDER_H_ */
