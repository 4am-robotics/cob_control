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
 *   This header contains the interface description of constraints
 *
 ****************************************************************/

#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_

#include "cob_twist_controller/augmented_solver_data_types.h"
#include "cob_twist_controller/constraints/constraint_base.h"

/* BEGIN ConstraintsBuilder *************************************************************************************/
/// Class providing a static method to create constraints.
template <typename PRIO = uint32_t>
class ConstraintsBuilder
{
    public:
        static std::set<tConstraintBase> create_constraints(AugmentedSolverParams &augmentedSolverParams,
                                                            const KDL::JntArray& q,
                                                            const Matrix6Xd &jacobianData,
                                                            const Eigen::Vector3d& eePosition);

    private:
        ConstraintsBuilder() {}
        ~ConstraintsBuilder() {}
};
/* END ConstraintsBuilder ***************************************************************************************/

/* BEGIN ConstraintParamsCA *************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsCA : public ConstraintParamsBase
{
    public:

        ConstraintParamsCA(const AugmentedSolverParams& params, const Matrix6Xd &jacobianData, const Eigen::Vector3d& eePosition)
        : ConstraintParamsBase(params), jacobianData_(jacobianData), eePosition_(eePosition)
        {}

        virtual ~ConstraintParamsCA()
        {}

        Matrix6Xd getEndeffectorJacobian() const
        {
            return this->jacobianData_;
        }

        Eigen::Vector3d getEndeffectorPosition() const
        {
            return this->eePosition_;
        }


    private:
        const Matrix6Xd& jacobianData_;
        const Eigen::Vector3d& eePosition_;


};
/* END ConstraintParamsCA ***************************************************************************************/

/* BEGIN CollisionAvoidance *************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename PRIO = uint32_t>
class CollisionAvoidance : public ConstraintBase<PRIO>
{
    public:

        CollisionAvoidance(PRIO prio, const KDL::JntArray& q) : ConstraintBase<PRIO>(prio, q)
        {
        }

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getSafeRegion() const;
        virtual Eigen::VectorXd getPartialValues() const;
        virtual double getSelfMotionMagnitude(Eigen::MatrixXd& particularSolution, Eigen::MatrixXd& homogeneousSolution) const;

        virtual ~CollisionAvoidance()
        {
        }
};
/* END CollisionAvoidance ***************************************************************************************/

/* BEGIN ConstraintParamsJLA ************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsJLA : public ConstraintParamsBase
{
    public:

        ConstraintParamsJLA(const AugmentedSolverParams& params) : ConstraintParamsBase(params)
        {}

        virtual ~ConstraintParamsJLA()
        {}

};
/* END ConstraintParamsJLA **************************************************************************************/

/* BEGIN JointLimitAvoidance ************************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename PRIO = uint32_t>
class JointLimitAvoidance : public ConstraintBase<PRIO>
{
    public:

        JointLimitAvoidance(PRIO prio, const KDL::JntArray& q) : ConstraintBase<PRIO>(prio, q)
        {
        }

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getSafeRegion() const;
        virtual Eigen::VectorXd getPartialValues() const;
        virtual double getSelfMotionMagnitude(Eigen::MatrixXd& particularSolution, Eigen::MatrixXd& homogeneousSolution) const;

        double getValue(Eigen::VectorXd steps) const;

        virtual ~JointLimitAvoidance()
        {
        }

};
/* END JointLimitAvoidance **************************************************************************************/

/* BEGIN JointLimitAvoidanceMid *********************************************************************************/
/// Class providing methods that realize a CollisionAvoidance constraint.
template <typename PRIO = uint32_t>
class JointLimitAvoidanceMid : public ConstraintBase<PRIO>
{
    public:

        JointLimitAvoidanceMid(PRIO prio, const KDL::JntArray& q) : ConstraintBase<PRIO>(prio, q)
        {
        }

        virtual double getValue() const;
        virtual double getDerivativeValue() const;
        virtual double getSafeRegion() const;
        virtual Eigen::VectorXd getPartialValues() const;
        virtual double getSelfMotionMagnitude(Eigen::MatrixXd& particularSolution, Eigen::MatrixXd& homogeneousSolution) const;

        virtual ~JointLimitAvoidanceMid()
        {
        }
};
/* END JointLimitAvoidanceMid ***********************************************************************************/


#include "cob_twist_controller/constraints/constraint_impl.h" // implementation of templated class

#endif /* CONSTRAINT_H_ */
