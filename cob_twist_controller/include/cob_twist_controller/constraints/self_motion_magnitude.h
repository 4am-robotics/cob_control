/*
 * self_motion_magnitude.h
 *
 *  Created on: May 21, 2015
 *      Author: fxm-mb
 */

#ifndef SELF_MOTION_MAGNITUDE_H_
#define SELF_MOTION_MAGNITUDE_H_

#include "cob_twist_controller/cob_twist_controller_data_types.h"

#include "ros/ros.h"

#include <vector>
#include <algorithm>

class SelfMotionMagnitudeDeterminatorBase
{
    public:
        SelfMotionMagnitudeDeterminatorBase()
        {

        }

        virtual ~SelfMotionMagnitudeDeterminatorBase()
        {

        }

        virtual double calculate(const InvDiffKinSolverParams& params, const Eigen::MatrixXd& particular_solution,
                                 const Eigen::MatrixXd& homogeneous_solution) const = 0;
};

template
<bool MAXIMIZE>
class SmmDeterminatorVelocityBounds : public SelfMotionMagnitudeDeterminatorBase
{
    public:
        SmmDeterminatorVelocityBounds()
        {

        }

        virtual ~SmmDeterminatorVelocityBounds() {}

        virtual double calculate(const InvDiffKinSolverParams& params, const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
        {
            std::vector<double> velLim = params.limits_vel;
            uint16_t cntRows = particular_solution.rows();
            if (cntRows != homogeneous_solution.rows() || cntRows != velLim.size())
            {
                ROS_ERROR("Count of rows do not match for particular solution, homogeneous solution and vector limits.");
                ROS_ERROR("Part.Solution = %d\nHom.Solution = %d\velLim = %d\n", cntRows, (int) homogeneous_solution.rows(), (int) velLim.size());
                return 0.0;
            }

            double kMax;
            double kMin;
            double kResult = 0.0;
            for (uint16_t i = 0; i < cntRows; ++i)
            {
                double upper = (velLim[i] - particular_solution(i)) / homogeneous_solution(i);
                double lower = (-velLim[i] - particular_solution(i)) / homogeneous_solution(i);

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

            if(kMax > kMin)
            {
                if(MAXIMIZE)
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
                ROS_ERROR("The requested end-effector velocity is too high. A proper k cannot be found! Assuming 0.0 for deactivation. ");
            }



            return kResult;
        }
};


class SmmDeterminatorConstant : public SelfMotionMagnitudeDeterminatorBase
{
    public:
        SmmDeterminatorConstant()
        {

        }

        virtual ~SmmDeterminatorConstant() {}

        virtual double calculate(const InvDiffKinSolverParams& params, const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution) const
        {
            return params.kappa;
        }
};


/// Abstract base class defining interfaces for the creation of a specific solver.
template <typename T>
class SelfMotionMagnitudeFactory
{
    public:

        static double calculate(const InvDiffKinSolverParams& params, const Eigen::MatrixXd& particular_solution, const Eigen::MatrixXd& homogeneous_solution)
        {
            T* cs = getInstance();
            double k = cs->calculate(params, particular_solution, homogeneous_solution);
            return k;
        }

    protected:

        static T* instance_;

        /**
         * The interface method to create a specific solver to solve the inverse kinematics problem.
         * @param asParams References the augmented solver parameters.
         * @param jacobianData References the current Jacobian (matrix data only).
         * @param jacobianDataTransposed References the current Jacobian transpose (matrix data only).
         * @return A specific solver.
         */
        static T* getInstance()
        {
            if (NULL == instance_)
            {
                instance_ = new T();
            }

            return instance_;
        }
};

template <typename T>
T* SelfMotionMagnitudeFactory<T>::instance_ = NULL;

#endif /* SELF_MOTION_MAGNITUDE_H_ */
