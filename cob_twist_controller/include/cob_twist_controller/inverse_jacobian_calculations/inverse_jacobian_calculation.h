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
 *   Declaration of pseudoinverse calculation classes.
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_H
#define COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_H

#include "cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation_base.h"

/* BEGIN PInvBySVD **********************************************************************************************/
class PInvBySVD : public IPseudoinverseCalculator
{
    public:
        /** Implementation of calculate member
         * See base for more information on parameters
         */
        virtual Eigen::MatrixXd calculate(const Eigen::MatrixXd& jacobian) const;

        /** Implementation of calculate member
         * See base for more information on parameters
         */
        virtual Eigen::MatrixXd calculate(const TwistControllerParams& params,
                                          boost::shared_ptr<DampingBase> db,
                                          const Eigen::MatrixXd& jacobian) const;

        virtual ~PInvBySVD() {}
};
/* END PInvBySVD ************************************************************************************************/

/* BEGIN PInvDirect **********************************************************************************************/
class PInvDirect : public IPseudoinverseCalculator
{
    public:
        /** Implementation of calculate member
         * See base for more information on parameters
         */
        virtual Eigen::MatrixXd calculate(const Eigen::MatrixXd& jacobian) const;

        /** Implementation of calculate member
         * See base for more information on parameters
         */
        virtual Eigen::MatrixXd calculate(const TwistControllerParams& params,
                                          boost::shared_ptr<DampingBase> db,
                                          const Eigen::MatrixXd& jacobian) const;

        virtual ~PInvDirect() {}
};
/* END PInvDirect ************************************************************************************************/

#endif  // COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_H
