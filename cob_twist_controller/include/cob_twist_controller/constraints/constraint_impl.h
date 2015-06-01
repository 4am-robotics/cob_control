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
 * \date Date of creation: May, 2015
 *
 * \brief
 *   Implementation of several constraints
 *
 ****************************************************************/

#ifndef CONSTRAINT_IMPL_H_
#define CONSTRAINT_IMPL_H_

#include "cob_twist_controller/constraints/constraint.h"
#include "ros/ros.h"
#include "cob_collision_object_publisher/ObjectOfInterest.h"

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>

/* BEGIN ConstraintsBuilder *************************************************************************************/
/**
 * Static builder method to create damping methods dependent on parameterization.
 */
template <typename PRIO>
std::set<tConstraintBase> ConstraintsBuilder<PRIO>::create_constraints(AugmentedSolverParams &augmentedSolverParams,
                                                                       const KDL::JntArray& q,
                                                                       const Matrix6Xd &jacobianData,
                                                                       const Eigen::Vector3d& eePosition)
{
    std::set<tConstraintBase> constraints;

    tConstraintBase jla;
    switch (augmentedSolverParams.constraint)
    {
        case GPM_JLA:
            jla.reset(new JointLimitAvoidance<PRIO>(100, q)); // TODO: take care PRIO could be of different type than UINT32
            jla->setConstraintParams(new ConstraintParamsJLA(augmentedSolverParams));
            constraints.insert(jla);
            break;
        case GPM_JLA_MID:
            jla.reset(new JointLimitAvoidanceMid<PRIO>(100, q)); // TODO: take care PRIO could be of different type than UINT32
            jla->setConstraintParams(new ConstraintParamsJLA(augmentedSolverParams)); // same params as for normal JLA
            constraints.insert(jla);
            break;
        case GPM_CA:
            jla.reset(new CollisionAvoidance<PRIO>(50, q)); // TODO: take care PRIO could be of different type than UINT32
            jla->setConstraintParams(new ConstraintParamsCA(augmentedSolverParams, jacobianData, eePosition));
            constraints.insert(jla);
            break;
        default:
            ROS_ERROR("Constraint %d not available for GPM", augmentedSolverParams.constraint);
    }


//    ConstraintBase<PRIO> *db = NULL;
//    switch(augmentedSolverParams.damping_method)
//    {
//        case NONE:
//            db = new DampingNone(augmentedSolverParams);
//            break;
//        case CONSTANT:
//            db = new DampingConstant(augmentedSolverParams);
//            break;
//        case MANIPULABILITY:
//            db = new DampingManipulability(augmentedSolverParams);
//            break;
//        default:
//            ROS_ERROR("DampingMethod %d not defined! Aborting!", augmentedSolverParams.damping_method);
//            break;
//    }

    return constraints;
}
/* END ConstraintsBuilder *******************************************************************************************/

/* BEGIN CollisionAvoidance *************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename PRIO>
double CollisionAvoidance<PRIO>::getValue() const
{

    return 0.0;
}

template <typename PRIO>
double CollisionAvoidance<PRIO>::getDerivativeValue() const
{
    return 0.0;
}

template <typename PRIO>
double CollisionAvoidance<PRIO>::getSafeRegion() const
{
    return 0.1; // in [m]
}


template <typename PRIO>
Eigen::VectorXd  CollisionAvoidance<PRIO>::getPartialValues() const
{
    uint8_t vecRows = static_cast<uint8_t>(this->jointPos_.rows());
    Eigen::VectorXd partialValues = Eigen::VectorXd::Zero(vecRows);

    // Create virtual obstacle
//    double stepSize = 0.01;
//    Eigen::Vector3d obstaclePosition(0.5, -0.3, 1);// [m]
//    Eigen::Vector3d obstacleDirectionA(0, 1, 0);
//    Eigen::Vector3d obstacleDirectionB(0, 0, 1);


//    // Calculate the nearest obstacle
//    // v_x = v_x_0 + s * v_a + t * v_b;
    const ConstraintParamsCA* constrParamCAPtr = reinterpret_cast<const ConstraintParamsCA*>(this->constraintParams_);
    const AugmentedSolverParams &params = constrParamCAPtr->getAugmentedSolverParams();
//    Eigen::Vector3d endeffectorPosition = constrParamCAPtr->getEndeffectorPosition();
//    Eigen::Vector3d obstVector;
//    //OS_INFO_STREAM("endeffectorPosition: " << std::endl << endeffectorPosition);
//    bool found = false;
//    double dist;
//    for (double s = 0.0; s < 1.0; s = s + stepSize)
//    {
//        for (double t = 0.0; t < 1.0; t = t + stepSize)
//        {
//            Eigen::Vector3d tmpObstVector = obstaclePosition + s * obstacleDirectionA + t * obstacleDirectionB;
//            Eigen::Vector3d distVec = endeffectorPosition - tmpObstVector;
//            dist = distVec.norm();
//
//
//            //ROS_INFO_STREAM("tmpObstVector: " << std::endl << tmpObstVector);
//            //ROS_INFO_STREAM("Calculated distance: " << dist);
//
//            if (this->getSafeRegion() > dist)
//            {
//                obstVector = tmpObstVector;
//                found = true;
//                break;
//            }
//        }
//
//        if(found)
//        {
//            break;
//        }
//    }

    if (ros::service::exists("/getSmallestDistance", true))
    {
        ROS_INFO_STREAM("Service seems to exist!!!");

        Eigen::Vector3d endeffectorPosition = constrParamCAPtr->getEndeffectorPosition();
        cob_collision_object_publisher::ObjectOfInterest ooi;

        ooi.request.p.position.x = endeffectorPosition(0);
        ooi.request.p.position.y = endeffectorPosition(1);
        ooi.request.p.position.z = endeffectorPosition(2);
        ooi.request.p.orientation.w = 1.0;

        ooi.request.shapeType = 2; // visualization_msgs::Marker::SPHERE

        //ROS_INFO_STREAM("Calling service");
        bool found = ros::service::call("/getSmallestDistance", ooi);
        //ROS_INFO_STREAM("Service returned" << found);

        if (found)
        {

            KDL::Jacobian new_jac_chain(7); // TODO: Change hard coded
            KDL::JntArray ja = this->jointPos_;
            //params.jnt2jac.JntToJac(ja, new_jac_chain); // TODO: Change hard coded

            params.jnt2jac->JntToJac(ja, new_jac_chain, 4);

            double dist = ooi.response.distance;

            if (this->getSafeRegion() > dist)
            {
                Eigen::Vector3d obstVector;
                obstVector <<  ooi.response.obstacle.position.x,  ooi.response.obstacle.position.y, ooi.response.obstacle.position.z;

                const ConstraintParamsCA* constrParamCAPtr = reinterpret_cast<const ConstraintParamsCA*>(this->constraintParams_);
                //Matrix6Xd jac = constrParamCAPtr->getEndeffectorJacobian();

                Matrix6Xd jac = new_jac_chain.data;

                Eigen::Vector3d endeffectorPosition = constrParamCAPtr->getEndeffectorPosition();
                Eigen::Matrix3Xd m = Eigen::Matrix3Xd::Zero(3, jac.cols());

                m << jac.row(0),
                     jac.row(1),
                     jac.row(2);

                // m.conservativeResize(3, 4);

                Eigen::Vector3d distVector = endeffectorPosition - obstVector;

                if (dist > 0.0)
                {
                    partialValues =  2.0 * ((dist - this->getSafeRegion()) / dist) * m.transpose() * distVector;
                }
                else
                {
                    partialValues =  2.0 * (((1.0e-9) - this->getSafeRegion()) / (1.0e-9)) * m.transpose() * distVector;
                }


                ROS_INFO_STREAM("Costfunction results: " << pow( (dist - this->getSafeRegion()), 2.0));
                ROS_INFO_STREAM("jac: " << std::endl << jac);
            }
        }
    }



    return partialValues;
}

template <typename PRIO>
double CollisionAvoidance<PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const
{
    const AugmentedSolverParams &params = this->constraintParams_->getAugmentedSolverParams();
    //return SelfMotionMagnitudeFactory< SmmDeterminatorVelocityBounds<MAX_CRIT> >::calculate(params, particularSolution, homogeneousSolution);
    return params.kappa;
}
/* END CollisionAvoidance ***************************************************************************************/

/* BEGIN JointLimitAvoidance ************************************************************************************/
/// Class providing methods that realize a JLA constraint.
template <typename PRIO>
double JointLimitAvoidance<PRIO>::getValue(Eigen::VectorXd steps) const
{
    const AugmentedSolverParams &params = this->constraintParams_->getAugmentedSolverParams();
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;
    double H_q = 0.0;
    for(uint8_t i = 0; i < this->jointPos_.rows() ; ++i)
    {
        double jntPosWithStep = this->jointPos_(i) + steps(i);
        double nom = pow(limits_max[i] - limits_min[i], 2.0);
        double denom = (limits_max[i] - jntPosWithStep) * (jntPosWithStep - limits_min[i]);
        H_q += nom / denom;
    }

    H_q = H_q / 4.0;
    return H_q;
}

template <typename PRIO>
double JointLimitAvoidance<PRIO>::getValue() const
{
    return this->getValue(Eigen::VectorXd::Zero(this->jointPos_.rows()));
}

template <typename PRIO>
double JointLimitAvoidance<PRIO>::getDerivativeValue() const
{
    return 0.0;
}

template <typename PRIO>
double JointLimitAvoidance<PRIO>::getSafeRegion() const
{
    return 0.0;
}

template <typename PRIO>
double JointLimitAvoidance<PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const
{
    // k_H by Armijo-Rule
    double t;
    const AugmentedSolverParams &params = this->constraintParams_->getAugmentedSolverParams();
//    Eigen::VectorXd gradient = this->getPartialValues();
//    double costFunctionVal = this->getValue();
//
//    double l = 0.0;
//    double t = 1.0;
//    double beta = 0.9;
//
//    // Eigen::VectorXd d = params.kappa * gradient;
//    Eigen::VectorXd d = -1.0 * gradient;
//
//    Eigen::VectorXd new_d = t * d;
//    double nextCostFunctionVal = this->getValue(new_d);
//
//    double summand = t * gradient.transpose() * d;
//
//    while(nextCostFunctionVal > (costFunctionVal + summand))
//    {
//        double t_new = pow(beta, l + 1.0);
//
//        if (!( (t_new >= (0.0001 * t)) && (t_new <= (0.9999 * t))))
//        {
//            ROS_ERROR("Outside of valid t range. Break!!!");
//            t = t_new;
//            break;
//        }
//
//        t = t_new;
//        l = l + 1.0;
//
//        new_d = t * d;
//        nextCostFunctionVal = this->getValue(new_d);
//        summand = t * gradient.transpose() * d;
//    }


    t = SelfMotionMagnitudeFactory< SmmDeterminatorVelocityBounds<MIN_CRIT> >::calculate(params, particularSolution, homogeneousSolution);
    return t;
}

template <typename PRIO>
Eigen::VectorXd JointLimitAvoidance<PRIO>::getPartialValues() const
{
    const AugmentedSolverParams &params = this->constraintParams_->getAugmentedSolverParams();
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;

    double rad = M_PI / 180.0;
    uint8_t vecRows = static_cast<uint8_t>(this->jointPos_.rows());
    Eigen::VectorXd partialValues = Eigen::VectorXd::Zero(vecRows);

    //ROS_WARN_STREAM("limits_min: " << std::endl << limits_min << std::endl);
    //ROS_WARN_STREAM("limits_max: " << std::endl << limits_max << std::endl);



    for(uint8_t i = 0; i < this->jointPos_.rows() ; ++i)
    {
        partialValues(i) = 0.0; // in the else cases -> output always 0
        //See Chan paper ISSN 1042-296X [Page 288]
        double minDelta = (this->jointPos_(i) - limits_min[i]);
        double maxDelta = (limits_max[i] - this->jointPos_(i));

//        ROS_WARN_STREAM("joint pos: " << this->jointPos_(i) << std::endl);
//        ROS_WARN_STREAM("minDelta: " << minDelta << std::endl);
//        ROS_WARN_STREAM("maxDelta: " << maxDelta << std::endl);
//
//        ROS_WARN_STREAM("limits_min: " << std::endl << limits_min[i] << std::endl);
//        ROS_WARN_STREAM("limits_max: " << std::endl << limits_max[i] << std::endl);

//        if( minDelta * maxDelta < 0.0)
//        {
//            ROS_ERROR_STREAM("LIMITING NECESSARY for joint : " << static_cast<unsigned int>(i) << std::endl);

            double nominator = (2.0 * this->jointPos_(i) - limits_min[i] - limits_max[i]) * (limits_max[i] - limits_min[i]) * (limits_max[i] - limits_min[i]);
            double denom = 4.0 * minDelta * minDelta * maxDelta * maxDelta;
            partialValues(i) = nominator / denom;
//        }
    }

    //double k = params.kappa;
    return partialValues;
}
/* END JointLimitAvoidance **************************************************************************************/

/* BEGIN 2nd JointLimitAvoidance ************************************************************************************/
/// Class providing methods that realize a joint limit avoidance constraint (trying to stay next to the middle between limits).
template <typename PRIO>
double JointLimitAvoidanceMid<PRIO>::getValue() const
{
    return 0.0;
}

template <typename PRIO>
double JointLimitAvoidanceMid<PRIO>::getDerivativeValue() const
{
    return 0.0;
}

template <typename PRIO>
double JointLimitAvoidanceMid<PRIO>::getSafeRegion() const
{
    return 0.0;
}

/// Method proposed by Liegeois
template <typename PRIO>
Eigen::VectorXd JointLimitAvoidanceMid<PRIO>::getPartialValues() const
{
    const AugmentedSolverParams &params = this->constraintParams_->getAugmentedSolverParams();
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;

    double rad = M_PI / 180.0;
    uint8_t vecRows = static_cast<uint8_t>(this->jointPos_.rows());
    Eigen::VectorXd partialValues = Eigen::VectorXd::Zero(vecRows);

    for(uint8_t i = 0; i < vecRows; ++i)
    {
        double minDelta = (this->jointPos_(i) - limits_min[i]);
        double maxDelta = (limits_max[i] - this->jointPos_(i));

//        ROS_WARN_STREAM("joint pos: " << this->jointPos_(i) << std::endl);
//        ROS_WARN_STREAM("minDelta: " << minDelta << std::endl);
//        ROS_WARN_STREAM("maxDelta: " << maxDelta << std::endl);
//
//        ROS_WARN_STREAM("limits_min: " << std::endl << limits_min[i] << std::endl);
//        ROS_WARN_STREAM("limits_max: " << std::endl << limits_max[i] << std::endl);

        if( minDelta * maxDelta < 0.0)
        {
            ROS_WARN_STREAM("Limit of joint " << int(i) << " reached: " << std::endl
                            << "pos=" << this->jointPos_(i) << ";lim_min=" << limits_min[i] << ";lim_max=" << limits_max[i]);
        }

        //Liegeois method can also be found in Chan paper ISSN 1042-296X [Page 288]
        double limits_mid = 1.0 / 2.0 * (limits_max[i] + limits_min[i]);
        double nominator = this->jointPos_(i) - limits_mid;
        double denom = limits_max[i] - limits_min[i];
        partialValues(i) = nominator / denom;
    }

    double k = params.kappa;
    return k * partialValues;
}

template <typename PRIO>
double JointLimitAvoidanceMid<PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const
{
    const AugmentedSolverParams &params = this->constraintParams_->getAugmentedSolverParams();
    return SelfMotionMagnitudeFactory<SmmDeterminatorVelocityBounds<MAX_CRIT> >::calculate(params, particularSolution, homogeneousSolution);
}
/* END 2nd JointLimitAvoidance **************************************************************************************/

#endif /* CONSTRAINT_IMPL_H_ */
