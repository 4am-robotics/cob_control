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
 *   This header contains the interface description of damping methods
 *
 ****************************************************************/
#ifndef DAMPING_MANIPULABILITY_H_
#define DAMPING_MANIPULABILITY_H_

#include "damping_base.h"
#include "cob_twist_controller/augmented_solver_data_types.h"

class DampingManipulability : public DampingBase
{
    public:
        DampingManipulability(AugmentedSolverParams &asSolverParams, Matrix6Xd &jacobianData);

        ~DampingManipulability() {}

        virtual double get_damping_factor() const;
};

#endif /* DAMPING_MANIPULABILITY_H_ */
