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
 *   This package provides the implementation of an inverse kinematics solver.
 *
 ****************************************************************/
#include "cob_twist_controller/augmented_solver.h"
#include "cob_twist_controller/constraint_solvers/constraint_solver_factory_builder.h"

//#include <Eigen/QR>
//#include <Eigen/LU>

#include "ros/ros.h"

int AugmentedSolver::CartToJnt(const KDL::JntArray& q_in,
                               const KDL::JntArray& last_q_dot,
                               const KDL::Twist& v_in,
                               const KDL::Frame &base_position,
                               const KDL::Frame &chain_base,
                               const Eigen::VectorXd &tracking_errors,
                               KDL::JntArray& qdot_out)
{
    int8_t retStat = -1;
    this->adjustJac(q_in, base_position, chain_base);
    Eigen::VectorXd v_in_vec = Eigen::VectorXd::Zero(this->jac_.rows());

    //ROS_INFO_STREAM_NAMED("AugmentedSolver::CartToJnt", "ColPivQR: Rank of Jacobian: " << this->jac_.data.colPivHouseholderQr().rank() << std::endl);
    //ROS_INFO_STREAM_NAMED("AugmentedSolver::CartToJnt", "FullPivLu: Rank of Jacobian: " << this->jac_.data.fullPivLu().rank() << std::endl);

    ///convert input
    for (int i=0; i < this->jac_.rows(); ++i)
    {
        v_in_vec(i) = v_in(i);
    }

    Eigen::MatrixXd qdot_out_vec;
    retStat = ConstraintSolverFactoryBuilder::calculateJointVelocities(this->params_,
                                                                       this->jac_.data,
                                                                       v_in_vec,
                                                                       q_in,
                                                                       last_q_dot,
                                                                       tracking_errors,
                                                                       qdot_out_vec);

    ///convert output
    for(int i = 0; i < jac_.columns(); i++)
    {
        qdot_out(i) = qdot_out_vec(i);
    }

    return retStat;
}

/**
 * Adjustment for the Jacobian in case of base gets active or not.
 * If base_active new columns will be added here.
 */
void AugmentedSolver::adjustJac(const KDL::JntArray& q_in,
                                const KDL::Frame &base_position,
                                const KDL::Frame &chain_base)
{
    ///Let the ChainJntToJacSolver calculate the jacobian "jac_chain" for the current joint positions "q_in"
    KDL::Jacobian jac_chain(chain_.getNrOfJoints());
    Eigen::Matrix<double,6,3> jac_b;
    jnt2jac_.JntToJac(q_in, jac_chain);
    if(params_.base_active)
    {
        Eigen::Matrix<double, 3, 3> chain_base_rot, base_rot, tip_base_rot;
        Eigen::Vector3d w_chain_base;
        Eigen::Vector3d r_chain_base;
        Eigen::Vector3d tangential_vel;

        double base_ratio = params_.base_ratio;

        //Create standard platform jacobian
        jac_b.setZero();

        // Get current x and y position from EE and chain_base with respect to base_footprint
        Eigen::Vector3d r_base_link(base_position.p.x(),
                                    base_position.p.y(),
                                    base_position.p.z());

        chain_base_rot <<     chain_base.M.data[0],chain_base.M.data[1],chain_base.M.data[2],
                        chain_base.M.data[3],chain_base.M.data[4],chain_base.M.data[5],
                        chain_base.M.data[6],chain_base.M.data[7],chain_base.M.data[8];

        // Transform from base_link to chain_base
        Eigen::Vector3d w_base_link(0,0,base_ratio);
        //Eigen::Vector3d w_base_link(0,0,1);
        w_chain_base = chain_base_rot*w_base_link;
        r_chain_base = chain_base_rot*r_base_link;

        //Calculate tangential velocity
        tangential_vel = w_chain_base.cross(r_chain_base);

         //Vx-Base <==> q8 effects a change in the following chain_base Vx velocities
        jac_b(0,0) = base_ratio*chain_base_rot(0,0);
        jac_b(0,1) = base_ratio*chain_base_rot(0,1);
        jac_b(0,2) = tangential_vel(0);

        // Vy-Base <==> q9 effects a change in the following chain_base Vy velocities
        jac_b(1,0) = base_ratio*chain_base_rot(1,0);
        jac_b(1,1) = base_ratio*chain_base_rot(1,1);
        jac_b(1,2) = tangential_vel(1);

        // Vz-Base <==>  effects a change in the following chain_base Vz velocities
        jac_b(2,0) = base_ratio*chain_base_rot(2,0);
        jac_b(2,1) = base_ratio*chain_base_rot(2,1);
        jac_b(2,2) = tangential_vel(2);

        //Phi <==> Wz with respect to base_link
        jac_b(3,2) = w_chain_base(0);
        jac_b(4,2) = w_chain_base(1);
        jac_b(5,2) = w_chain_base(2);

        //combine chain Jacobian and platform Jacobian
        Eigen::Matrix<double, 6, Eigen::Dynamic> jac_full;
        jac_full.resize(6,chain_.getNrOfJoints() + jac_b.cols());
        jac_full << jac_chain.data,jac_b;
        jac_.resize(chain_.getNrOfJoints() + jac_b.cols());
        jac_.data << jac_full;
    }
    else
    {
        jac_.resize(chain_.getNrOfJoints());
        jac_.data << jac_chain.data;
    }
}
