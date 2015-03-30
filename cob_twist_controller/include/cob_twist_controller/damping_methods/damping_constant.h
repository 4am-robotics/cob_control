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
 *   This header contains the description of the constant damping method.
 *
 ****************************************************************/
#ifndef DAMPING_CONSTANT_H_
#define DAMPING_CONSTANT_H_

#include "damping_base.h"
#include "cob_twist_controller/augmented_solver_data_types.h"

class DampingConstant : public DampingBase
{
    public:
        DampingConstant(AugmentedSolverParams &asSolverParams, Matrix6Xd &jacobianData);

        ~DampingConstant() {}

        virtual double get_damping_factor() const;
};

#endif /* DAMPING_CONSTANT_H_ */
