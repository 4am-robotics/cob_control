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
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This header contains the template implementations for
 *   self motion magnitude determination (factor k_H for GPM)
 *   One proposal according to:
 *   Euler J.A., Dubey R.V., Babcock S.M. (1989),
 *   "Self motion determination based on actuator velocity bounds
 *   for redundant manipulators". J. Robotic Syst., 6: 417-425
 *
 *   Other:
 *   Parameter from dynamic_reconfigure.
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_CONSTRAINTS_SELF_MOTION_MAGNITUDE_H
#define COB_TWIST_CONTROLLER_CONSTRAINTS_SELF_MOTION_MAGNITUDE_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

#include "cob_twist_controller/cob_twist_controller_data_types.h"

class SelfMotionMagnitudeDeterminatorBase
{
    public:
        SelfMotionMagnitudeDeterminatorBase()
        {}

        virtual ~SelfMotionMagnitudeDeterminatorBase()
        {}

        virtual double calculate(const LimiterParams& params,
                                 const Eigen::MatrixXd& particular_solution,
                                 const Eigen::MatrixXd& homogeneous_solution) const = 0;
};

template
<bool MAXIMIZE>
class SmmDeterminatorVelocityBounds : public SelfMotionMagnitudeDeterminatorBase
{
    public:
        SmmDeterminatorVelocityBounds()
        {}

        virtual ~SmmDeterminatorVelocityBounds()
        {}

        /// Implementation of SMM. Formula: See header comment!
        virtual double calculate(const LimiterParams& params, const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
        {
            uint16_t cntRows = particular_solution.rows();

            if (cntRows != homogeneous_solution.rows() || cntRows != params.limits_vel.size())
            {
                ROS_ERROR("Count of rows do not match for particular solution, homogeneous solution and vector limits.");
                ROS_ERROR("Part.Solution = %d\nHom.Solution = %d\velLim = %d\n", cntRows, (int) homogeneous_solution.rows(), (int) params.limits_vel.size());
                return 0.0;
            }

            if (homogeneous_solution.norm() <= ZERO_THRESHOLD)
            {
                return 0.0;
            }

            double kMax = -1.0;
            double kMin = 1.0;
            double kResult = 0.0;
            for (uint16_t i = 0; i < cntRows; ++i)
            {
                double upper;
                double lower;
                if (std::fabs(static_cast<double>(homogeneous_solution(i))) > ZERO_THRESHOLD)
                {
                    upper = (params.limits_vel[i] - particular_solution(i)) / homogeneous_solution(i);
                    lower = (-params.limits_vel[i] - particular_solution(i)) / homogeneous_solution(i);
                }

                if (0 == i)
                {
                    kMax = std::max(upper, lower);
                    kMin = std::min(upper, lower);
                }
                else
                {
                    kMax = std::min(std::max(upper, lower), kMax);
                    kMin = std::max(std::min(upper, lower), kMin);
                }
            }

            if (kMax > kMin)
            {
                if (MAXIMIZE)
                {
                    kResult = kMax;
                    ROS_INFO_STREAM("Calculated MAX k = " << kResult);
                }
                else
                {
                    kResult = kMin;
                    ROS_INFO_STREAM("Calculated MIN k = " << kResult);
                }
            }
            else
            {
                ROS_ERROR("The requested end-effector velocity is too high. A proper k cannot be found! Assuming MIN: -1.0 or MAX 1.0. ");
                kResult = MAXIMIZE ? 1.0 : -1.0;
            }

            return kResult;
        }
};

class SmmDeterminatorConstant : public SelfMotionMagnitudeDeterminatorBase
{
    public:
        SmmDeterminatorConstant()
        {}

        virtual ~SmmDeterminatorConstant()
        {}

        virtual double calculate(const LimiterParams& params,
                                 const Eigen::MatrixXd& particular_solution,
                                 const Eigen::MatrixXd& homogeneous_solution) const
        {
            // not really used anymore
            return 1.0;
        }
};

/// Factory to create an SMM type object and call calculate method on it.
template <typename T>
class SelfMotionMagnitudeFactory
{
    public:
        static double calculate(const LimiterParams& params,
                                const Eigen::MatrixXd& particular_solution,
                                const Eigen::MatrixXd& homogeneous_solution)
        {
            T smm_determinator;
            return smm_determinator.calculate(params, particular_solution, homogeneous_solution);
        }
};

#endif  // COB_TWIST_CONTROLLER_CONSTRAINTS_SELF_MOTION_MAGNITUDE_H
