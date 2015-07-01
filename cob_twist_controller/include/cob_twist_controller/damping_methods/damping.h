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
#ifndef DAMPING_H_
#define DAMPING_H_

#include "cob_twist_controller/damping_methods/damping_base.h"

/* BEGIN DampingBuilder *****************************************************************************************/
/// Class providing a static method to create damping method objects.
class DampingBuilder
{
    public:
        static DampingBase* create_damping(const InvDiffKinSolverParams &params);

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
        DampingNone(const InvDiffKinSolverParams &params)
        : DampingBase(params)
        {}

        ~DampingNone() {}

        virtual double get_damping_factor(const Eigen::VectorXd &sorted_singular_values,
                                          const Eigen::MatrixXd &jacobian_data) const;
};
/* END DampingNone **********************************************************************************************/

/* BEGIN DampingConstant ****************************************************************************************/
/// Class implementing a method to return the constant factor.
class DampingConstant : public DampingBase
{
    public:
        DampingConstant(const InvDiffKinSolverParams &params)
        : DampingBase(params)
        {}

        ~DampingConstant() {}

        virtual double get_damping_factor(const Eigen::VectorXd &sorted_singular_values,
                                          const Eigen::MatrixXd &jacobian_data) const;
};
/* END DampingConstant ******************************************************************************************/

/* BEGIN DampingManipulability **********************************************************************************/
/// Class implementing a method to return a factor corresponding to the measure of manipulability.
class DampingManipulability : public DampingBase
{
    public:
        DampingManipulability(const InvDiffKinSolverParams &params)
        : DampingBase(params)
        {}

        ~DampingManipulability() {}

        virtual double get_damping_factor(const Eigen::VectorXd &sorted_singular_values,
                                          const Eigen::MatrixXd &jacobian_data) const;
};
/* END DampingManipulability ************************************************************************************/

/* BEGIN DampingLeastSingularValues **********************************************************************************/
/// Class implementing a method to return a damping factor based on least singular value.
class DampingLeastSingularValues : public DampingBase
{
    public:
        DampingLeastSingularValues(const InvDiffKinSolverParams &params)
        : DampingBase(params)
        {}

        ~DampingLeastSingularValues() {}

        virtual double get_damping_factor(const Eigen::VectorXd &sorted_singular_values,
                                          const Eigen::MatrixXd &jacobian_data) const;
};
/* END DampingLeastSingularValues ************************************************************************************/

#endif /* DAMPING_H_ */
