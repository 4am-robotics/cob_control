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
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(augmentedSolverParams, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<tJla > jla(new tJla(100, q, params));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if(GPM_JLA_MID == augmentedSolverParams.constraint)
    {
        typedef JointLimitAvoidanceMid<ConstraintParamsJLA, PRIO> tJlaMid;
        // same params as for normal JLA
        ConstraintParamsJLA params = ConstraintParamFactory<ConstraintParamsJLA>::createConstraintParams(augmentedSolverParams, data_mediator);
        // TODO: take care PRIO could be of different type than UINT32
        boost::shared_ptr<tJlaMid > jla(new tJlaMid(100, q, params));
        constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(jla));
    }
    else if(GPM_CA == augmentedSolverParams.constraint)
    {
        typedef CollisionAvoidance<ConstraintParamsCA, PRIO> tCollisionAvoidance;
        uint32_t available_dists = data_mediator.obstacleDistancesCnt();
        uint32_t startPrio = 100;
        for (uint32_t i = 0; i < available_dists; ++i)
        {
            ConstraintParamsCA params = ConstraintParamFactory<ConstraintParamsCA>::createConstraintParams(augmentedSolverParams, data_mediator);
            // TODO: take care PRIO could be of different type than UINT32
            boost::shared_ptr<tCollisionAvoidance > ca(new tCollisionAvoidance(startPrio--, q, params, jnt_to_jac));
            constraints.insert(boost::static_pointer_cast<PriorityBase<PRIO> >(ca));
        }
    }
    else
    {
        // Nothing to do here!
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
double CollisionAvoidance<T_PARAMS, PRIO>::getActivationThreshold() const
{
    return 0.1; // in [m]
}

template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd  CollisionAvoidance<T_PARAMS, PRIO>::getPartialValues() const
{
    uint8_t vecRows = static_cast<uint8_t>(this->jointPos_.rows());
    Eigen::VectorXd partialValues = Eigen::VectorXd::Zero(vecRows);
    const AugmentedSolverParams& params = this->constraintParams_.getAugmentedSolverParams();
    int size_of_frames = params.frame_names.size();
    Distance d = this->constraintParams_.current_distance_;
    if (this->getActivationThreshold() > d.min_distance)
    {
        std::vector<std::string>::const_iterator str_it = std::find(params.frame_names.begin(),
                                                                    params.frame_names.end(),
                                                                    d.frame_id);
        if (params.frame_names.end() != str_it)
        {
            uint32_t pos = str_it - params.frame_names.begin();
            uint32_t frame_number = pos + 1;
            KDL::Jacobian new_jac_chain(size_of_frames);
            KDL::JntArray ja = this->jointPos_;
            this->jnt_to_jac_.JntToJac(ja, new_jac_chain, frame_number);

            Eigen::Matrix3Xd m = Eigen::Matrix3Xd::Zero(3, size_of_frames);

            m << new_jac_chain.data.row(0),
                 new_jac_chain.data.row(1),
                 new_jac_chain.data.row(2);

            if (d.min_distance > 0.0)
            {
                partialValues =  2.0 * ((d.min_distance - this->getActivationThreshold()) / d.min_distance) * m.transpose() * d.distance_vec;
            }
        }
    }

    return partialValues;
}

template <typename T_PARAMS, typename PRIO>
double CollisionAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const
{
    const AugmentedSolverParams &params = this->constraintParams_.getAugmentedSolverParams();
    //return SelfMotionMagnitudeFactory< SmmDeterminatorVelocityBounds<MAX_CRIT> >::calculate(params, particularSolution, homogeneousSolution);
    return params.kappa;
}
/* END CollisionAvoidance ***************************************************************************************/

/* BEGIN JointLimitAvoidance ************************************************************************************/
/// Class providing methods that realize a JLA constraint.
template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getValue(Eigen::VectorXd steps) const
{
    const AugmentedSolverParams &params = this->constraintParams_.getAugmentedSolverParams();
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
double JointLimitAvoidance<T_PARAMS, PRIO>::getActivationThreshold() const
{
    return 0.0;
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidance<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const
{
    // k_H by Armijo-Rule
    double t;
    const AugmentedSolverParams &params = this->constraintParams_.getAugmentedSolverParams();
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
    const AugmentedSolverParams &params = this->constraintParams_.getAugmentedSolverParams();
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;
    double rad = M_PI / 180.0;
    uint8_t vec_rows = static_cast<uint8_t>(this->jointPos_.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(vec_rows);
    for(uint8_t i = 0; i < this->jointPos_.rows() ; ++i)
    {
        partial_values(i) = 0.0; // in the else cases -> output always 0
        //See Chan paper ISSN 1042-296X [Page 288]
        double min_delta = (this->jointPos_(i) - limits_min[i]);
        double max_delta = (limits_max[i] - this->jointPos_(i));
        double nominator = (2.0 * this->jointPos_(i) - limits_min[i] - limits_max[i]) * (limits_max[i] - limits_min[i]) * (limits_max[i] - limits_min[i]);
        double denom = 4.0 * min_delta * min_delta * max_delta * max_delta;
        partial_values(i) = nominator / denom;

    }

    return partial_values;
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
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getActivationThreshold() const
{
    return 0.0;
}

/// Method proposed by Liegeois
template <typename T_PARAMS, typename PRIO>
Eigen::VectorXd JointLimitAvoidanceMid<T_PARAMS, PRIO>::getPartialValues() const
{
    const AugmentedSolverParams &params = this->constraintParams_.getAugmentedSolverParams();
    std::vector<double> limits_min = params.limits_min;
    std::vector<double> limits_max = params.limits_max;

    double rad = M_PI / 180.0;
    uint8_t vec_rows = static_cast<uint8_t>(this->jointPos_.rows());
    Eigen::VectorXd partial_values = Eigen::VectorXd::Zero(vec_rows);

    for(uint8_t i = 0; i < vec_rows; ++i)
    {
        double min_delta = (this->jointPos_(i) - limits_min[i]);
        double max_delta = (limits_max[i] - this->jointPos_(i));
        if( min_delta * max_delta < 0.0)
        {
            ROS_WARN_STREAM("Limit of joint " << int(i) << " reached: " << std::endl
                            << "pos=" << this->jointPos_(i) << ";lim_min=" << limits_min[i] << ";lim_max=" << limits_max[i]);
        }

        //Liegeois method can also be found in Chan paper ISSN 1042-296X [Page 288]
        double limits_mid = 1.0 / 2.0 * (limits_max[i] + limits_min[i]);
        double nominator = this->jointPos_(i) - limits_mid;
        double denom = limits_max[i] - limits_min[i];
        partial_values(i) = nominator / denom;
    }

    double k = params.kappa;
    return k * partial_values;
}

template <typename T_PARAMS, typename PRIO>
double JointLimitAvoidanceMid<T_PARAMS, PRIO>::getSelfMotionMagnitude(const Eigen::MatrixXd& particularSolution, const Eigen::MatrixXd& homogeneousSolution) const
{
    const AugmentedSolverParams &params = this->constraintParams_.getAugmentedSolverParams();
    return SelfMotionMagnitudeFactory<SmmDeterminatorVelocityBounds<MAX_CRIT> >::calculate(params, particularSolution, homogeneousSolution);
}
/* END 2nd JointLimitAvoidance **************************************************************************************/

#endif /* CONSTRAINT_IMPL_H_ */
