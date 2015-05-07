/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
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
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This package provides the definitions of an inverse kinematics solver.
 *
 ****************************************************************/
#ifndef AUGMENTED_SOLVER_H
#define AUGMENTED_SOLVER_H

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Core>

#include "cob_twist_controller/augmented_solver_data_types.h"

/**
* Implementation of a inverse velocity kinematics algorithm based
* on the generalize pseudo inverse to calculate the velocity
* transformation from Cartesian to joint space of a general
* KDL::Chain. It uses a svd-calculation based on householders
* rotations.
*
* @ingroup KinematicFamily
*/
class AugmentedSolver
{
public:
    /**
     * Constructor of the solver
     *
     * @param chain the chain to calculate the inverse velocity
     * kinematics for
     * @param eps if a singular value is below this value, its
     * inverse is set to zero, default: 0.00001
     * @param maxiter maximum iterations for the svd calculation,
     * default: 150
     *
     */
    AugmentedSolver(const KDL::Chain& chain, double eps=0.001) :
        chain_(chain),
        jac_(chain_.getNrOfJoints()),
        jnt2jac_(chain_)
    {
    }

    virtual ~AugmentedSolver() {};
    
    /** CartToJnt for chain using SVD including base and various DampingMethods **/
    virtual int CartToJnt(const KDL::JntArray& q_in,
                          const KDL::JntArray& last_q_dot,
                          const KDL::Twist& v_in,
                          const KDL::Frame &base_position,
                          const KDL::Frame &chain_base,
                          const Eigen::VectorXd &tracking_errors,
                          KDL::JntArray& qdot_out);

    inline virtual int CartToJnt(const KDL::JntArray& q_in,
                                 const KDL::JntArray& last_q_dot,
                                 const KDL::Twist& v_in,
                                 const Eigen::VectorXd &tracking_errors,
                                 KDL::JntArray& qdot_out)
    {
        KDL::Frame dummy;
        dummy.p = KDL::Vector(0,0,0);
        dummy.M = KDL::Rotation::Quaternion(0,0,0,0);
        return CartToJnt(q_in, last_q_dot, v_in, dummy, dummy, tracking_errors, qdot_out);
    }

    inline void SetAugmentedSolverParams(AugmentedSolverParams params)
    {
        params_ = params;
    }

    inline AugmentedSolverParams GetAugmentedSolverParams()
    {
        return params_;
    }

private:
    const KDL::Chain chain_;
    KDL::Jacobian jac_, jac_base_;
    KDL::ChainJntToJacSolver jnt2jac_;
    AugmentedSolverParams params_;

    /**
     * Adjustment of the member Jacobian
     * @param q_in Input joint positions.
     * @param base_position Current base position.
     * @param chain_base Current frame of the chain base.
     */
    void adjustJac(const KDL::JntArray& q_in, const KDL::Frame &base_position, const KDL::Frame &chain_base);
};
#endif
