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
 * \date Date of creation: May, 2015
 *
 * \brief
 *   This header contains the interface description of constraint solvers
 *   Pure virtual methods have to be implemented in subclasses
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_BASE_H
#define COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_BASE_H

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include "cob_twist_controller/damping_methods/damping_base.h"
#include "cob_twist_controller/cob_twist_controller_data_types.h"

class IPseudoinverseCalculator
{
    public:
        /**
         * Pure virtual method for calculation of the pseudoinverse
         * @param jacobian The Jacobi matrix.
         * @return A pseudoinverse Jacobian
         */
        virtual Eigen::MatrixXd calculate(const Eigen::MatrixXd& jacobian) const = 0;

        /**
         * Pure virtual method for calculation of the pseudoinverse (allows to consider damping and truncation)
         * @param params The parameters from parameter server.
         * @param db The damping method.
         * @param jacobian The Jacobi matrix.
         * @return A pseudoinverse Jacobian
         */
        virtual Eigen::MatrixXd calculate(const TwistControllerParams& params,
                                          boost::shared_ptr<DampingBase> db,
                                          const Eigen::MatrixXd& jacobian) const = 0;

        /**
         * Class has no members so implementing an empty destructor.
         */
        virtual ~IPseudoinverseCalculator() {}
};

#endif  // COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_BASE_H
