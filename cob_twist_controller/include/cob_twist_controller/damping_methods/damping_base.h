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
using namespace Eigen;
/// Base class for solvers, defining interface methods.
class DampingBase
{
    public:
        virtual double get_damping_factor() const = 0;

        virtual VectorXd calc_damped_singulars(VectorXd sortedSingValues) const
        {
            // small change to ref: here quadratic damping due to Control of Redundant Robot Manipulators : R.V. Patel, 2005, Springer [Page 13-14]
            VectorXd singularValuesInv = VectorXd::Zero(sortedSingValues.rows());
            double lambda = this->get_damping_factor();
            for(uint32_t i = 0; i < sortedSingValues.rows(); ++i)
            {
                double denominator = (sortedSingValues(i) * sortedSingValues(i) + pow(lambda, 2) );
                singularValuesInv(i) = (denominator < this->asParams_.eps) ? 0.0 : sortedSingValues(i) / denominator;
            }

            return singularValuesInv;
        }

        virtual ~DampingBase() {}

    protected:
        DampingBase(AugmentedSolverParams &asParams, Matrix6Xd &jacobianData) : asParams_(asParams), jacobianData_(jacobianData) { }

        const AugmentedSolverParams &asParams_;
        const Matrix6Xd &jacobianData_;
};

#endif /* DAMPING_METHOD_INTERFACE_H_ */
