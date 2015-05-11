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
 *   This header contains the description of the unconstraint solver
 *   Implements methods from constraint_solver_base
 *
 ****************************************************************/
#ifndef GRADIENT_PROJECTION_METHOD_SOLVER_H_
#define GRADIENT_PROJECTION_METHOD_SOLVER_H_

#include <set>
#include "ros/ros.h"

#include "cob_twist_controller/augmented_solver_data_types.h"
#include "cob_twist_controller/constraint_solvers/solvers/constraint_solver_base.h"


#include "cob_twist_controller/constraints/constraint_base.h"
#include "cob_twist_controller/constraints/constraint.h"

#include <boost/shared_ptr.hpp>


typedef boost::shared_ptr<ConstraintBase<> > tConstraintBase;

class GradientProjectionMethodSolver : public ConstraintSolver<>
{
    public:

        /**
         * Specific implementation of solve-method to solve IK problem without any constraints.
         * See base class ConstraintSolver for more details on params and returns.
         */
        virtual Eigen::MatrixXd solve(const Eigen::VectorXd &inCartVelocities,
                                      const KDL::JntArray& q,
                                      const KDL::JntArray& last_q_dot,
                                      const Eigen::VectorXd &tracking_errors) const;

        virtual void init()
        {
            tConstraintBase ca(new CollisionAvoidance<>(this->asParams_, 100)); // TODO: LAter remove currently only for test
            tConstraintBase jla(new JointLimitAvoidance<>(this->asParams_, 99)); // TODO: LAter remove currently only for test

            registerConstraint(ca);
            registerConstraint(jla);
        }

        GradientProjectionMethodSolver(AugmentedSolverParams &asParams,
                           Matrix6Xd &jacobianData)
                           : ConstraintSolver(asParams,
                                              jacobianData)
        {
        }

        virtual ~GradientProjectionMethodSolver()
        {
            this->clearConstraints();
        }

        void registerConstraint(tConstraintBase constraint)
        {
            this->constraints_.insert(constraint);
        }

        void clearConstraints()
        {
            this->constraints_.clear(); // calls destructor on all objects and clears the set
        }

    protected:
        std::set<tConstraintBase> constraints_; // set inserts sorted (default less operator); if element has already been added it returns an iterator on it.
};

#endif /* GRADIENT_PROJECTION_METHOD_SOLVER_H_ */
