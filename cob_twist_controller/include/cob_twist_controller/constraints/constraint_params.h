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
 * \date Date of creation: June, 2015
 *
 * \brief
 *   Implementation of parameter classes for constraints.
 *
 ****************************************************************/
#ifndef CONSTRAINT_PARAMS_H_
#define CONSTRAINT_PARAMS_H_

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/augmented_solver_data_types.h"

/* BEGIN ConstraintParamsBase ***********************************************************************************/
/// Base class for all derived parameter classes.
class ConstraintParamsBase
{
    public:

        ConstraintParamsBase(const AugmentedSolverParams& params) : params_(params)
        {}

        ~ConstraintParamsBase()
        {}

        const AugmentedSolverParams& getAugmentedSolverParams() const
        {
            return this->params_;
        }

    protected:
        const AugmentedSolverParams& params_;
};
/* END ConstraintParamsBase *************************************************************************************/

/* BEGIN ConstraintParamsCA *************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsCA : public ConstraintParamsBase
{
    public:

        Distance current_distance_;

        ConstraintParamsCA(const AugmentedSolverParams& params)
        : ConstraintParamsBase(params)
        {}

        ConstraintParamsCA(const ConstraintParamsCA& cpca)
        : ConstraintParamsBase(cpca.params_)
        {
            current_distance_ = cpca.current_distance_;
        }

        virtual ~ConstraintParamsCA()
        {}

};
/* END ConstraintParamsCA ***************************************************************************************/

/* BEGIN ConstraintParamsJLA ************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsJLA : public ConstraintParamsBase
{
    public:

        ConstraintParamsJLA(const AugmentedSolverParams& params)
        : ConstraintParamsBase(params)
        {}

        ConstraintParamsJLA(const ConstraintParamsJLA& cpjla)
        : ConstraintParamsBase(cpjla.params_)
        {}

        virtual ~ConstraintParamsJLA()
        {}

};
/* END ConstraintParamsJLA **************************************************************************************/

typedef boost::shared_ptr<ConstraintParamsBase> tConstraintParamsBase;

#endif /* CONSTRAINT_PARAMS_H_ */
