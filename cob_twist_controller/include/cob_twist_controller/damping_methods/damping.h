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
 *   This header contains the class and method definitions of
 *   several damping methods.
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_DAMPING_METHODS_DAMPING_H
#define COB_TWIST_CONTROLLER_DAMPING_METHODS_DAMPING_H

#include "cob_twist_controller/damping_methods/damping_base.h"

/* BEGIN DampingBuilder *****************************************************************************************/
/// Class providing a static method to create damping method objects.
class DampingBuilder
{
    public:
        static DampingBase* createDamping(const TwistControllerParams& params);

    private:
        DampingBuilder() {}
        ~DampingBuilder() {}
};
/* END DampingBuilder *******************************************************************************************/

/* BEGIN DampingNone ****************************************************************************************/
/// Class implementing a method to return the constant factor.
class DampingNone : public DampingBase
{
    public:
        explicit DampingNone(const TwistControllerParams& params)
        : DampingBase(params)
        {}

        ~DampingNone() {}

        virtual Eigen::MatrixXd getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                 const Eigen::MatrixXd& jacobian_data) const;
};
/* END DampingNone **********************************************************************************************/

/* BEGIN DampingConstant ****************************************************************************************/
/// Class implementing a method to return the constant factor.
class DampingConstant : public DampingBase
{
    public:
        explicit DampingConstant(const TwistControllerParams& params)
        : DampingBase(params)
        {}

        ~DampingConstant() {}

        virtual Eigen::MatrixXd getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                 const Eigen::MatrixXd& jacobian_data) const;
};
/* END DampingConstant ******************************************************************************************/

/* BEGIN DampingManipulability **********************************************************************************/
/// Class implementing a method to return a factor corresponding to the measure of manipulability.
class DampingManipulability : public DampingBase
{
    public:
        explicit DampingManipulability(const TwistControllerParams& params)
        : DampingBase(params)
        {}

        ~DampingManipulability() {}

        virtual Eigen::MatrixXd getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                 const Eigen::MatrixXd& jacobian_data) const;
};
/* END DampingManipulability ************************************************************************************/

/* BEGIN DampingLeastSingularValues **********************************************************************************/
/// Class implementing a method to return a damping factor based on least singular value.
class DampingLeastSingularValues : public DampingBase
{
    public:
        explicit DampingLeastSingularValues(const TwistControllerParams& params)
        : DampingBase(params)
        {}

        ~DampingLeastSingularValues() {}

        virtual Eigen::MatrixXd getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                 const Eigen::MatrixXd& jacobian_data) const;
};
/* END DampingLeastSingularValues ************************************************************************************/

/* BEGIN DampingSigmoid **********************************************************************************/
/// Class implementing a method to return a damping factor based on a sigmoid function.
class DampingSigmoid : public DampingBase
{
    public:
        explicit DampingSigmoid(const TwistControllerParams& params)
        : DampingBase(params)
        {}

        ~DampingSigmoid() {}

        virtual Eigen::MatrixXd getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                                 const Eigen::MatrixXd& jacobian_data) const;
};
/* END DampingSigmoid ************************************************************************************/

#endif  // COB_TWIST_CONTROLLER_DAMPING_METHODS_DAMPING_H
