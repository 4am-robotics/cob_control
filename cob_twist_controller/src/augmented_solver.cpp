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

#include "cob_twist_controller/chainfk_solvers/advanced_chainfksolverpos_recursive.h"



#include "cob_obstacle_distance/ObjectOfInterest.h"


int AugmentedSolver::CartToJnt(const KDL::JntArray& q_in,
                               const KDL::JntArray& last_q_dot,
                               const KDL::Twist& v_in,
                               const KDL::Frame &base_position,
                               const KDL::Frame &chain_base,
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

    AdvancedChainFkSolverPos_recursive adChnFkSolverPos(chain_);

    KDL::Frame pOut;
    int retVal = adChnFkSolverPos.JntToCart(q_in, pOut);

    KDL::Frame joint4pos = adChnFkSolverPos.getPostureAtJnt(3);


    Eigen::Matrix<double, 3, 3> chain_base_rot, base_pos_rot;

    chain_base_rot <<     chain_base.M.data[0],chain_base.M.data[1],chain_base.M.data[2],
                    chain_base.M.data[3],chain_base.M.data[4],chain_base.M.data[5],
                    chain_base.M.data[6],chain_base.M.data[7],chain_base.M.data[8];

    base_pos_rot << joint4pos.M.data[0], joint4pos.M.data[1],joint4pos.M.data[2],
            joint4pos.M.data[3],joint4pos.M.data[4],joint4pos.M.data[5],
            joint4pos.M.data[6],joint4pos.M.data[7],joint4pos.M.data[8];


    Eigen::Vector3d base2chainbaseTranslation(chain_base.p.x(),
                                              chain_base.p.y(),
                                              chain_base.p.z());
    Eigen::Vector3d baseLink2armBase = (-1) * chain_base_rot.inverse() * base2chainbaseTranslation;
    //ROS_INFO_STREAM("ROTATED base2chainbaseTranslation: " << baseLink2armBase << std::endl);


    // Get current x and y position from EE and chain_base with respect to base_footprint
    Eigen::Vector3d chainbase2fourthJntPos(joint4pos.p.x(),
                                           joint4pos.p.y(),
                                           joint4pos.p.z());

    //Eigen::Vector3d testo2 = chain_base_rot * base_pos_rot.inverse() * chainbase2fourthJntPos;
    Eigen::Vector3d armBase2JntPos = chain_base_rot.inverse() * chainbase2fourthJntPos;
    //ROS_INFO_STREAM("ROTATED chainbase2fourthJntPos: " << armBase2JntPos << std::endl);


    Eigen::Vector3d eePosition = baseLink2armBase + armBase2JntPos;

    //ROS_INFO_STREAM("base2chainbaseTranslation: " << base2chainbaseTranslation << std::endl);
    //ROS_INFO_STREAM("chainbase2fourthJntPos: " << chainbase2fourthJntPos << std::endl);
    ROS_INFO_STREAM("eePosition: " << eePosition << std::endl);


    // ROS_INFO_STREAM("Endeffector position: " << std::endl << eePosition);

    Eigen::MatrixXd qdot_out_vec;

    this->params_.jnt2jac = &jnt2jac_;
    retStat = ConstraintSolverFactoryBuilder::calculateJointVelocities(this->params_,
                                                                       this->jac_.data,
                                                                       v_in_vec,
                                                                       q_in,
                                                                       last_q_dot,
                                                                       eePosition,
                                                                       qdot_out_vec);

    if (ros::service::exists("/getSmallestDistance", true))
    {
        cob_obstacle_distance::ObjectOfInterest ooi;

        ooi.request.p.position.x = eePosition(0);
        ooi.request.p.position.y = eePosition(1);
        ooi.request.p.position.z = eePosition(2);
        ooi.request.p.orientation.w = 1.0;

        ooi.request.shapeType = 2; // visualization_msgs::Marker::SPHERE

        bool found = ros::service::call("/getSmallestDistance", ooi);
    }

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


    //ROS_INFO_STREAM("adChnFkSolverPos retVal: " << retVal << std::endl);
    //adChnFkSolverPos.dumpAllJntPostures();



    KDL::Jacobian jac_chain(chain_.getNrOfJoints());


    //KDL::Jacobian jac_chain2(chain_.getNrOfJoints());

    //KDL::Jacobian jac_chain3(chain_.getNrOfJoints());


    Eigen::Matrix<double,6,3> jac_b;

    // ROS_INFO_STREAM("Nr of segments: " << std::endl << chain_.getNrOfSegments() << std::endl);

    jnt2jac_.JntToJac(q_in, jac_chain);
    // ROS_INFO_STREAM("jac_chain.data: " << std::endl << jac_chain.data << std::endl);

    //jnt2jac_.JntToJac(q_in, jac_chain2, 7);
    //ROS_INFO_STREAM("jac_chain2_joint7.data: " << std::endl << jac_chain2.data << std::endl);

    //jnt2jac_.JntToJac(q_in, jac_chain3, 4);
    //ROS_INFO_STREAM("jac_chain3_joint4.data: " << std::endl << jac_chain3.data << std::endl);


    KDL::Frame pos;

    //ROS_INFO_STREAM("Other Endeffector position: " << std::endl << pos.p.x() << std::endl << pos.p.y() << std::endl << pos.p.z());

    // KDL::Jacobian jac_chain_2(jac_chain);

//    KDL::JntArray q_tmp_in(3);
//
//    for(unsigned int i = 0; i < q_tmp_in.rows(); i++)
//    {
//        q_tmp_in(i) = q_in(i);
//        ROS_INFO_STREAM("q_tmp_in: " << q_tmp_in(i));
//        ROS_INFO_STREAM("q_in: " << q_in(i));
//    }

    // int retVal = jnt2jac_.JntToJac(q_tmp_in, jac_chain_2);


//    Eigen::Matrix<double, 6, Eigen::Dynamic> jac_blubby;
//    jac_blubby.resize(6, chain_.getNrOfJoints() - 3);
//    jac_blubby << jac_chain_2.data;


    // ROS_INFO_STREAM("Jac (6 x 7): " << std::endl << jac_chain_2.data);
    //jac_chain_2.data.conservativeResize(6, chain_.getNrOfJoints() - 3);

    // jac_chain_2.data << jac_blubby;
    //ROS_INFO_STREAM("Jac (6 x 4): " << std::endl << jac_chain_2.data);
    // ROS_INFO_STREAM("Return Value: " << retVal);


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
        jac_full << jac_chain.data, jac_b;
        jac_.resize(chain_.getNrOfJoints() + jac_b.cols());
        jac_.data << jac_full;
    }
    else
    {
        jac_.resize(chain_.getNrOfJoints());
        jac_.data << jac_chain.data;
    }
}
