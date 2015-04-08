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
#include "cob_twist_controller/constraint_solvers/solvers/joint_limit_avoidance_solver.h"

/**
 * Specific implementation of the solve method to avoid joint limits.
 * This is done by calculation of a weighting for the Jacobian.
 * Uses the base implementation of calculatePinvJacobianBySVD to calculate the pseudo-inverse (weighted) Jacobian.
 */
Eigen::MatrixXd JointLimitAvoidanceSolver::solve(const Eigen::VectorXd &inCartVelocities, const KDL::JntArray& q, const KDL::JntArray& last_q_dot) const
{
    uint32_t cols = this->jacobianData_.cols(); // in original implementation KDL::jacobian.columns has been taken!
    uint32_t rows = this->jacobianData_.rows(); // in original implementation KDL::jacobian.rows has been taken!
    Eigen::MatrixXd W_JLA = this->calculateWeighting(q, last_q_dot);
    // for the following formulas see Chan paper ISSN 1042-296X [Page 288]
    Eigen::MatrixXd root_W_JLA =  W_JLA.cwiseSqrt(); // element-wise sqrt -> ok because diag matrix W^(1/2)
    Eigen::MatrixXd inv_root_W_JLA =  root_W_JLA.inverse(); // -> W^(-1/2)

    // SVD of JLA weighted Jacobian: Damping will be done later in calculatePinvJacobianBySVD for pseudo-inverse Jacobian with additional truncation etc.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->jacobianData_ * inv_root_W_JLA,
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd weightedJacobianPseudoInverse = this->calculatePinvJacobianBySVD(svd);
    Eigen::MatrixXd qdots_out = weightedJacobianPseudoInverse * inCartVelocities;
    return qdots_out;
}

/**
 * This function calculates the weighting matrix used to penalize a joint when it is near and moving towards a limit.
 * The last joint velocity is used to determine if it that happens or not
 */
Eigen::MatrixXd JointLimitAvoidanceSolver::calculateWeighting(const KDL::JntArray& q, const KDL::JntArray& last_q_dot) const
{
    std::vector<double> limits_min = this->asSolverParams_.limits_min;
    std::vector<double> limits_max = this->asSolverParams_.limits_max;
    uint32_t cols = this->jacobianData_.cols();
    Eigen::VectorXd output = Eigen::VectorXd::Zero(cols);
    double rad = M_PI / 180.0;

    for(uint32_t i = 0; i < cols ; ++i)
    {
        output(i) = 1; // in the else cases -> output always 1
        if(i < q.rows()) // q has been initialized with chain_getNrOfJoints -> q.rows should be equal to getNrOfJoints.
        {
            //See Chan paper ISSN 1042-296X [Page 288]
            if( (last_q_dot(i) > 0.0 && ((limits_max[i] - q(i)) < (q(i) - limits_min[i])))
                    || (last_q_dot(i) < 0.0 && ((limits_max[i] - q(i)) > (q(i) - limits_min[i]))) ) // TODO: Condition: check with Felix ???
            {
                // calculation is only necessary in case of condition is true!
                double nominator = pow(limits_max[i]-limits_min[i], 2.0) * (2.0 * q(i) - limits_max[i] - limits_min[i]);
                double denominator = 4.0 * pow(limits_max[i] - q(i), 2.0) * pow(q(i) - limits_min[i], 2.0);
                if (denominator != 0.0)
                {
                    double partialPerformanceCriterion = rad * fabs(nominator / denominator);
                    output(i) = 1 + partialPerformanceCriterion;
                }
            }
        }
    }

    return output.asDiagonal();
}
