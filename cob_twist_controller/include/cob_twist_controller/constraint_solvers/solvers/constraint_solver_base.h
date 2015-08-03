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
 *   This header contains the interface description of constraint solvers
 *   Pure virtual methods have to be implemented in subclasses
 *
 ****************************************************************/
#ifndef CONSTRAINT_SOLVER_BASE_H_
#define CONSTRAINT_SOLVER_BASE_H_

#include <Eigen/Core>
#include <kdl/jntarray.hpp>
#include <boost/shared_ptr.hpp>
#include <cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation.h>
#include "cob_twist_controller/damping_methods/damping_base.h"
#include "cob_twist_controller/constraints/constraint_base.h"
#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/task_stack/task_stack_controller.h"

/// Base class for solvers, defining interface methods.
template <typename PINV = PInvBySVD>
class ConstraintSolver
{
    public:
        /**
         * The interface method to solve the inverse kinematics problem. Has to be implemented in inherited classes.
         * @param inCartVelocities The input velocities vector (in cartesian space).
         * @param joint_states The joint states with history.
         * @return The calculated new joint velocities.
         */
        virtual Eigen::MatrixXd solve(const Vector6d_t& in_cart_velocities,
                                      const JointStates& joint_states) = 0;

        /**
         * Inline method to set the damping
         * @param damping The new damping
         */
        inline void setDamping(boost::shared_ptr<DampingBase>& damping)
        {
            this->damping_ = damping;
        }

        /**
         * Method to initialize the solver if necessary
         */
        virtual void setConstraints(std::set<ConstraintBase_t>& constraints)
        {

        }

        /**
         * Method to initialize the solver if necessary
         */
        virtual void setJacobianData(const Matrix6Xd_t& jacobian_data)
        {
            this->jacobian_data_ = jacobian_data;
        }

        virtual ~ConstraintSolver() {}

    protected:

        ConstraintSolver(const TwistControllerParams& params, TaskStackController_t& task_stack_controller)
                         : params_(params), task_stack_controller_(task_stack_controller)
        {
        }

        const TwistControllerParams& params_; ///< References the inv. diff. kin. solver parameters.
        Matrix6Xd_t jacobian_data_; ///< References the current Jacobian (matrix data only).
        boost::shared_ptr<DampingBase> damping_; ///< The currently set damping method.
        PINV pinv_calc_; ///< An instance that helps solving the inverse of the Jacobian.
        TaskStackController_t& task_stack_controller_;
};

#endif /* CONSTRAINT_SOLVER_BASE_H_ */
