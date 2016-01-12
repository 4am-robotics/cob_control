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
 *   Implementation of an JLA solver.
 *   Special constraint: Avoid joint limits.
 *
 ****************************************************************/

#include <vector>
#include <ros/ros.h>

#include "cob_twist_controller/constraint_solvers/solvers/wln_joint_limit_avoidance_solver.h"

/**
 * This function calculates the weighting matrix used to penalize a joint when it is near and moving towards a limit.
 * The last joint velocity is used to determine if it that happens or not
 */
Eigen::MatrixXd WLN_JointLimitAvoidanceSolver::calculateWeighting(const JointStates& joint_states) const
{
    std::vector<double> limits_min = this->limiter_params_.limits_min;
    std::vector<double> limits_max = this->limiter_params_.limits_max;
    uint32_t cols = this->jacobian_data_.cols();
    Eigen::VectorXd weighting = Eigen::VectorXd::Zero(cols);

    KDL::JntArray q = joint_states.current_q_;
    KDL::JntArray q_dot = joint_states.current_q_dot_;

    for (uint32_t i = 0; i < cols ; ++i)
    {
        weighting(i) = 1.0;    // in the else cases -> weighting always 1
        if (i < q.rows())
        {
            // See Chan paper ISSN 1042-296X [Page 288]
            if ( (q_dot(i) > 0.0 && ((limits_max[i] - q(i)) < (q(i) - limits_min[i])))
                 || (q_dot(i) < 0.0 && ((limits_max[i] - q(i)) > (q(i) - limits_min[i]))) )
            {
                // calculation is only necessary in case of condition is true!
                double nominator = pow(limits_max[i]-limits_min[i], 2.0) * (2.0 * q(i) - limits_max[i] - limits_min[i]);
                double denominator = 4.0 * pow(limits_max[i] - q(i), 2.0) * pow(q(i) - limits_min[i], 2.0);
                if (denominator != 0.0)
                {
                    double partialPerformanceCriterion = fabs(nominator / denominator);
                    weighting(i) = 1 + partialPerformanceCriterion;
                }
            }
        }
    }

    return weighting.asDiagonal();
}
