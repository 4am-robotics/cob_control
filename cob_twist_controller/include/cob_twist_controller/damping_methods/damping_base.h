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
#ifndef DAMPING_METHOD_INTERFACE_H_
#define DAMPING_METHOD_INTERFACE_H_

#include "cob_twist_controller/augmented_solver_data_types.h"

/// Base class for solvers, defining interface methods.
class DampingBase
{
    public:
        virtual double get_damping_factor() const = 0;
        virtual ~DampingBase() = 0;

    protected:
        DampingBase(AugmentedSolverParams &asParams, Matrix6Xd &jacobianData);

        const AugmentedSolverParams &asParams_;
        const Matrix6Xd &jacobianData_;
};

#endif /* DAMPING_METHOD_INTERFACE_H_ */
