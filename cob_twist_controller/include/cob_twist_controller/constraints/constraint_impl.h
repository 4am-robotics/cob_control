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

#include <boost/pointer_cast.hpp>

#include <ros/ros.h>

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>

#include "cob_twist_controller/constraints/constraint.h"
#include "cob_twist_controller/constraints/constraint_params.h"
#include "cob_obstacle_distance/PartialValues.h"

template
<typename T>
class ConstraintParamFactory
{
    public:
        static boost::shared_ptr<T> createConstraintParams(AugmentedSolverParams &augmentedSolverParams,
                                         CallbackDataMediator& data_mediator)
        {
//            std::vector<boost::shared_ptr<T> > v;
//
//            boost::shared_ptr<T> params = boost::shared_ptr<T>(new T(augmentedSolverParams));
//            //T* params = new T(augmentedSolverParams);
//            data_mediator.fill(params);

            boost::shared_ptr<T> params = boost::shared_ptr<T>(new T(augmentedSolverParams));
            data_mediator.fill(params);
            return params;
        }

    private:
        ConstraintParamFactory()
        {}
};



/* BEGIN ConstraintsBuilder *************************************************************************************/
/**
 * Static builder method to create damping methods dependent on parameterization.
 */
template <typename PRIO>
std::set<tConstraintBase> ConstraintsBuilder<PRIO>::createConstraints(AugmentedSolverParams &augmentedSolverParams,
                                                                       const KDL::JntArray& q,
                                                                       const Matrix6Xd &jacobianData,
                                                                       KDL::ChainJntToJacSolver& jnt_to_jac,
                                                                       CallbackDataMediator& data_mediator)
{
    std::set<tConstraintBase> constraints;
    if (GPM_JLA == augmentedSolverParams.constraint)
    {
        typedef JointLimitAvoidance<ConstraintParamsJLA, PRIO> tJla;
        boost::shared_ptr<ConstraintParamsJLA> params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(augmentedSolverParams, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<tJla > jla(new tJla(100, q, params));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if(GPM_JLA_MID == augmentedSolverParams.constraint)
    {
        typedef JointLimitAvoidanceMid<ConstraintParamsJLA, PRIO> tJlaMid;
        // same params as for normal JLA
        boost::shared_ptr<ConstraintParamsJLA> params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(augmentedSolverParams, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<tJlaMid > jla(new tJlaMid(100, q, params));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if(GPM_CA == augmentedSolverParams.constraint)
    {
        typedef CollisionAvoidance<ConstraintParamsCA, PRIO> tCollisionAvoidance;
        uint32_t available_dists = data_mediator.obstacleDistancesCnt();

        ROS_INFO_STREAM(">>>>>>>>>>>>> FOUND available_dists: " << available_dists);

        uint32_t startPrio = 100;
        for (uint32_t i = 0; i < available_dists; ++i)
        {
            boost::shared_ptr<ConstraintParamsCA> params = ConstraintParamFactory<ConstraintParamsCA>::createConstraintParams(augmentedSolverParams, data_mediator);
            // TODO: take care PRIO could be of different type than UINT32
            boost::shared_ptr<tCollisionAvoidance > ca(new tCollisionAvoidance(startPrio--, q, params, jnt_to_jac));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(ca));
        }
    }
    else
    {
        ROS_ERROR("Constraint %d not available for GPM", augmentedSolverParams.constraint);
    }

    return constraints;
}
/* END ConstraintsBuilder *******************************************************************************************/

/* BEGIN CollisionAvoidance *************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getValue() const
{

    return 0.0;
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getDerivativeValue() const
{
    return 0.0;
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getSafeRegion() const
{
    return 0.1; // in [m]
}


//template <typename PRIO>
//Eigen::VectorXd  CollisionAvoidance<PRIO>::getPartialValues() const
//{
//    uint8_t vecRows = static_cast<uint8_t>(this->jointPos_.rows());
//    Eigen::VectorXd partialValues = Eigen::VectorXd::Zero(vecRows);
//
//    // Create virtual obstacle
////    double stepSize = 0.01;
////    Eigen::Vector3d obstaclePosition(0.5, -0.3, 1);// [m]
////    Eigen::Vector3d obstacleDirectionA(0, 1, 0);
////    Eigen::Vector3d obstacleDirectionB(0, 0, 1);
//
//
////    // Calculate the nearest obstacle
////    // v_x = v_x_0 + s * v_a + t * v_b;
//    const ConstraintParamsCA* constrParamCAPtr = reinterpret_cast<const ConstraintParamsCA*>(this->constraintParams_);
//    const AugmentedSolverParams &params = constrParamCAPtr->getAugmentedSolverParams();
////    Eigen::Vector3d endeffectorPosition = constrParamCAPtr->getEndeffectorPosition();
////    Eigen::Vector3d obstVector;
////    //OS_INFO_STREAM("endeffectorPosition: " << std::endl << endeffectorPosition);
////    bool found = false;
////    double dist;
////    for (double s = 0.0; s < 1.0; s = s + stepSize)
////    {
////        for (double t = 0.0; t < 1.0; t = t + stepSize)
////        {
////            Eigen::Vector3d tmpObstVector = obstaclePosition + s * obstacleDirectionA + t * obstacleDirectionB;
////            Eigen::Vector3d distVec = endeffectorPosition - tmpObstVector;
////            dist = distVec.norm();
////
////
////            //ROS_INFO_STREAM("tmpObstVector: " << std::endl << tmpObstVector);
////            //ROS_INFO_STREAM("Calculated distance: " << dist);
////
////            if (this->getSafeRegion() > dist)
////            {
////                obstVector = tmpObstVector;
////                found = true;
////                break;
////            }
////        }
////
////        if(found)
////        {
////            break;
////        }
////    }
//
//    if (ros::service::exists("/getSmallestDistance", true))
//    {
//        ROS_INFO_STREAM("Service seems to exist!!!");
//
//        Eigen::Vector3d endeffectorPosition = constrParamCAPtr->getEndeffectorPosition();
//        cob_obstacle_distance::ObjectOfInterest ooi;
//
//        ooi.request.p.position.x = endeffectorPosition(0);
//        ooi.request.p.position.y = endeffectorPosition(1);
//        ooi.request.p.position.z = endeffectorPosition(2);
//        ooi.request.p.orientation.w = 1.0;
//
//        ooi.request.shapeType = 2; // visualization_msgs::Marker::SPHERE
//
//        //ROS_INFO_STREAM("Calling service");
//        bool found = ros::service::call("/getSmallestDistance", ooi);
//        //ROS_INFO_STREAM("Service returned" << found);
//
//        if (found)
//        {
//
//            KDL::Jacobian new_jac_chain(7); // TODO: Change hard coded
//            KDL::JntArray ja = this->jointPos_;
//            //params.jnt2jac.JntToJac(ja, new_jac_chain); // TODO: Change hard coded
//
//            params.jnt2jac->JntToJac(ja, new_jac_chain, 4);
//
//            double dist = ooi.response.distance;
//
//            if (this->getSafeRegion() > dist)
//            {
//                Eigen::Vector3d obstVector;
//                obstVector <<  ooi.response.obstacle.position.x,  ooi.response.obstacle.position.y, ooi.response.obstacle.position.z;
//
//                const ConstraintParamsCA* constrParamCAPtr = reinterpret_cast<const ConstraintParamsCA*>(this->constraintParams_);
//                //Matrix6Xd jac = constrParamCAPtr->getEndeffectorJacobian();
//
//                Matrix6Xd jac = new_jac_chain.data;
//
//                Eigen::Vector3d endeffectorPosition = constrParamCAPtr->getEndeffectorPosition();
//                Eigen::Matrix3Xd m = Eigen::Matrix3Xd::Zero(3, jac.cols());
//
//                m << jac.row(0),
//                     jac.row(1),
//                     jac.row(2);
//
//                // m.conservativeResize(3, 4);
//
//                Eigen::Vector3d distVector = endeffectorPosition - obstVector;
//
//                if (dist > 0.0)
//                {
//                    partialValues =  2.0 * ((dist - this->getSafeRegion()) / dist) * m.transpose() * distVector;
//                }
//                else
//                {
//                    partialValues =  2.0 * (((1.0e-9) - this->getSafeRegion()) / (1.0e-9)) * m.transpose() * distVector;
//                }
//
//
//                ROS_INFO_STREAM("Costfunction results: " << pow( (dist - this->getSafeRegion()), 2.0));
//                ROS_INFO_STREAM("jac: " << std::endl << jac);
//            }
//        }
//    }
//
//
//
//    return partialValues;
//}

template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd  CollisionAvoidance<T_PARAMS, PRIO>::getPartialValues() const
{
    ROS_INFO_STREAM("CollisionAvoidance<T_PARAMS, PRIO>::getPartialValues()");
    uint8_t vecRows = static_cast<uint8_t>(this->jointPos_.rows());
    ROS_INFO_STREAM("uint8_t vecRows = static_cast<uint8_t>(this->jointPos_.rows());");
    Eigen::VectorXd partialValues = Eigen::VectorXd::Zero(vecRows);

    // Create virtual obstacle
//    double stepSize = 0.01;
//    Eigen::Vector3d obstaclePosition(0.5, -0.3, 1);// [m]
//    Eigen::Vector3d obstacleDirectionA(0, 1, 0);
//    Eigen::Vector3d obstacleDirectionB(0, 0, 1);


//    // Calculate the nearest obstacle
//    // v_x = v_x_0 + s * v_a + t * v_b;

    //boost::shared_ptr<ConstraintParamsCA> constrParamCAPtr = boost::static_pointer_cast<ConstraintParamsCA>(this->constraintParams_);
    //const ConstraintParamsCA* constrParamCAPtr = reinterpret_cast<const ConstraintParamsCA*>(this->constraintParams_);

    ROS_INFO_STREAM("Eigen::VectorXd partialValues = Eigen::VectorXd::Zero(vecRows);");
    ROS_INFO_STREAM("this->constraintParams_->current_distance_.min_distance: " << this->constraintParams_->current_distance_.min_distance);
    //const AugmentedSolverParams &params = this->constraintParams_->getAugmentedSolverParams();
    //AugmentedSolverParams params;
    //ROS_INFO_STREAM("AugmentedSolverParams params;");

    AugmentedSolverParams params = this->constraintParams_->getAugmentedSolverParams();
    ROS_INFO_STREAM("this->constraintParams_->getAugmentedSolverParams();");
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

    int size_of_jnts = params.joint_names.size();
    Distance d = this->constraintParams_->current_distance_;
    ROS_INFO_STREAM("Distance d = this->constraintParams_->current_distance_;");
    if (this->getSafeRegion() > d.min_distance)
    {
        std::vector<std::string>::const_iterator str_it = std::find(params.joint_names.begin(), params.joint_names.end(), d.frame_name);
        if (params.joint_names.end() != str_it)
        {

            int pos = str_it - params.joint_names.begin();
            ROS_INFO_STREAM("Found frame name: " << d.frame_name << " at pos: " << pos);
            int jnt_number = pos + 1;
            KDL::Jacobian new_jac_chain(size_of_jnts);
            KDL::JntArray ja = this->jointPos_;
            this->jnt_to_jac_.JntToJac(ja, new_jac_chain, jnt_number);

            Eigen::Matrix3Xd m = Eigen::Matrix3Xd::Zero(3, size_of_jnts);

            m << new_jac_chain.data.row(0),
                 new_jac_chain.data.row(1),
                 new_jac_chain.data.row(2);

            if (d.min_distance > 0.0)
            {
                partialValues =  2.0 * ((d.min_distance - this->getSafeRegion()) / d.min_distance) * m.transpose() * d.distance_vec;
                ROS_INFO_STREAM("Calculated partial values of " << d.frame_name << " : " << partialValues);
            }
        }
    }







//    if (ros::service::exists("/getAnotherSmallestDistance", true))
//    {
//        ROS_INFO_STREAM("Service seems to exist!!!");
//
//        Eigen::Vector3d endeffectorPosition; // = constrParamCAPtr->getEndeffectorPosition();
//        cob_obstacle_distance::PartialValues ooi;
//
//        //ooi.request.p.position.x = endeffectorPosition(0);
//        //ooi.request.p.position.y = endeffectorPosition(1);
//        //ooi.request.p.position.z = endeffectorPosition(2);
//        //ooi.request.p.orientation.w = 1.0;
//
//        ooi.request.shapeType = 2; // visualization_msgs::Marker::SPHERE
//        ooi.request.jntNumber = 4; // TODO: Hard Coded
//        ooi.request.activationDistance = 0.1; // TODO: Hard Coded
//
//        //ROS_INFO_STREAM("Calling service");
//        bool found = ros::service::call("/getAnotherSmallestDistance", ooi);
//        //ROS_INFO_STREAM("Service returned" << found);
//
//        if (found)
//        {
//
//            KDL::Jacobian new_jac_chain(7); // TODO: Change hard coded
//            KDL::JntArray ja = this->jointPos_;
//            this->jnt_to_jac_.JntToJac(ja, new_jac_chain);
////
////            params.jnt2jac->JntToJac(ja, new_jac_chain, 4);
////
////            double dist = ooi.response.distance;
////
////            if (this->getSafeRegion() > dist)
////            {
////                Eigen::Vector3d obstVector;
////                obstVector <<  ooi.response.obstacle.position.x,  ooi.response.obstacle.position.y, ooi.response.obstacle.position.z;
////
////                const ConstraintParamsCA* constrParamCAPtr = reinterpret_cast<const ConstraintParamsCA*>(this->constraintParams_);
////                //Matrix6Xd jac = constrParamCAPtr->getEndeffectorJacobian();
////
////                Matrix6Xd jac = new_jac_chain.data;
////
////                Eigen::Vector3d endeffectorPosition = constrParamCAPtr->getEndeffectorPosition();
////                Eigen::Matrix3Xd m = Eigen::Matrix3Xd::Zero(3, jac.cols());
////
////                m << jac.row(0),
////                     jac.row(1),
////                     jac.row(2);
////
////                // m.conservativeResize(3, 4);
////
////                Eigen::Vector3d distVector = endeffectorPosition - obstVector;
////
////                if (dist > 0.0)
////                {
////                    partialValues =  2.0 * ((dist - this->getSafeRegion()) / dist) * m.transpose() * distVector;
////                }
////                else
////                {
////                    partialValues =  2.0 * (((1.0e-9) - this->getSafeRegion()) / (1.0e-9)) * m.transpose() * distVector;
////                }
////
////
////                ROS_INFO_STREAM("Costfunction results: " << pow( (dist - this->getSafeRegion()), 2.0));
////                ROS_INFO_STREAM("jac: " << std::endl << jac);
////            }
//
//            int cnt = 0;
//            for(std::vector<double>::const_iterator it = ooi.response.partialValues.begin(); it != ooi.response.partialValues.end(); ++it)
//            {
//                partialValues(cnt++) = *it;
//            }
//
//        }
//    }



    return partialValues;
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const
{
    const AugmentedSolverParams &params = this->constraintParams_->getAugmentedSolverParams();
    //return SelfMotionMagnitudeFactory< SmmDeterminatorVelocityBounds<MAX_CRIT> >::calculate(params, particularSolution, homogeneousSolution);
    return params.kappa;
}
/* END CollisionAvoidance ***************************************************************************************/

/* BEGIN JointLimitAvoidance ************************************************************************************/
/// Class providing methods that realize a JLA constraint.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getValue(Eigen::VectorXd steps) const
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

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getValue() const
{
    return this->getValue(Eigen::VectorXd::Zero(this->jointPos_.rows()));
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getDerivativeValue() const
{
    return 0.0;
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getSafeRegion() const
{
    return 0.0;
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const
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

template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidance<T_PARAMS, PRIO>::getPartialValues() const
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
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getValue() const
{
    return 0.0;
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getDerivativeValue() const
{
    return 0.0;
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getSafeRegion() const
{
    return 0.0;
}

/// Method proposed by Liegeois
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidanceMid<T_PARAMS, PRIO>::getPartialValues() const
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

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const
{
    const AugmentedSolverParams &params = this->constraintParams_->getAugmentedSolverParams();
    return SelfMotionMagnitudeFactory<SmmDeterminatorVelocityBounds<MAX_CRIT> >::calculate(params, particularSolution, homogeneousSolution);
}
/* END 2nd JointLimitAvoidance **************************************************************************************/

#endif /* CONSTRAINT_IMPL_H_ */
