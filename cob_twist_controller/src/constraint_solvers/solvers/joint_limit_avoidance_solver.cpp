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

JointLimitAvoidanceSolver::JointLimitAvoidanceSolver(AugmentedSolverParams &asSolverParams,
                                                     Matrix6Xd &jacobianData,
                                                     Eigen::Transpose<Matrix6Xd> &jacobianDataTransposed)
    : ConstraintSolver(asSolverParams,
                       jacobianData,
                       jacobianDataTransposed)
{

}


JointLimitAvoidanceSolver::~JointLimitAvoidanceSolver()
{

}


Eigen::MatrixXd JointLimitAvoidanceSolver::solve(const Eigen::VectorXd &inCartVelocities, const KDL::JntArray& q, const KDL::JntArray& q_dot) const
{
    uint32_t cols = this->jacobianData_.cols(); // in original implementation KDL::jacobian.columns has been taken!
    uint32_t rows = this->jacobianData_.rows(); // in original implementation KDL::jacobian.rows has been taken!
    Eigen::MatrixXd W_JLA = this->calculateWeighting(q, q_dot);
    // SVD of JLA weighted Jacobian: Damping will be done later in calculatePinvJacobianBySVD for pseudo-inverse Jacobian with additional truncation etc.

    // TODO: Drüberschauen, weil hier einfach Damping Matrix aus SVD rausgenommen wurde und in calculatePinvJacobianBySVD hinzugefügt.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(this->jacobianData_ * W_JLA * this->jacobianDataTransposed_,
                                          Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd jacobianPseudoInverse = this->calculatePinvJacobianBySVD(svd);
    Eigen::MatrixXd qdots_out = W_JLA.inverse() * this->jacobianDataTransposed_ * jacobianPseudoInverse * inCartVelocities;
    return qdots_out;
}


Eigen::MatrixXd JointLimitAvoidanceSolver::calculateWeighting(const KDL::JntArray& q, const KDL::JntArray& q_dot) const
{
    //This function calculates the weighting matrix used to penalize a joint when it is near and moving towards a limit
    //The last joint velocity is used to determine if it that happens or not
    // return (diagonal) Weighting Matrix
    std::vector<double> limits_min = this->asSolverParams_.limits_min;
    std::vector<double> limits_max = this->asSolverParams_.limits_max;
    uint32_t cols = this->jacobianData_.cols();
    Eigen::VectorXd output = Eigen::VectorXd::Zero(cols);
    double rad = M_PI / 180.0;

    for(uint32_t i = 0; i < cols ; ++i)
    {
        output(i) = 1; // in the else cases -> output always 1
        // if(i < chain_.getNrOfJoints())
        if(i < q.rows()) // q has been initialized with chain_getNrOfJoints -> q.rows should be equal to getNrOfJoints.
        {
            //See Chan paper ISSN 1042-296X [Page 288]
            if( (q_dot(i) > 0 && ((limits_max[i] - q(i)) < (q(i) - limits_min[i])))
                    || (q_dot(i) < 0 && ((limits_max[i] - q(i)) > (q(i) - limits_min[i]))) ) // TODO: Condition: check with Felix ???
            {
                // calculation is only necessary in case of condition is true!
                double nominator = pow(limits_max[i]-limits_min[i], 2) * (2.0 * q(i) - limits_max[i] - limits_min[i]);
                double denominator = 4.0 * pow(limits_max[i] - q(i), 2) * pow(q(i) - limits_min[i], 2);
                if (denominator != 0.0)
                {
                    double partialPerformanceCriterion = rad * fabs(nominator / denominator);
                    output(i) = 1 + partialPerformanceCriterion;
                }

                //std::cout<<"partialPerformanceCriterion:"<<partialPerformanceCriterion<<std::endl;
            }
        }
    }

    return output.asDiagonal();
}
