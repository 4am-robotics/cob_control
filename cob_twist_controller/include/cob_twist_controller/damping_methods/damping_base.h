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

#include "cob_twist_controller/cob_twist_controller_data_types.h"

/// Base class for solvers, defining interface methods.
class DampingBase
{
    public:
        virtual ~DampingBase() {}
    
        virtual double getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                          const Eigen::MatrixXd& jacobian_data) const = 0;

    protected:
        DampingBase(const TwistControllerParams& params) : params_(params) { }

        const TwistControllerParams params_;
};

#endif /* DAMPING_METHOD_INTERFACE_H_ */
