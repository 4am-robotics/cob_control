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

#ifndef COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_PARAMS_H
#define COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_PARAMS_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include "cob_twist_controller/cob_twist_controller_data_types.h"

/* BEGIN ConstraintParamsBase ***********************************************************************************/
/// Base class for all derived parameter classes.
class ConstraintParamsBase
{
    public:
        ConstraintParamsBase(const TwistControllerParams& params,
                             const LimiterParams& limiter_params,
                             const std::string& id = std::string()) :
                tc_params_(params),
                limiter_params_(limiter_params),
                id_(id)
        {}

        ~ConstraintParamsBase()
        {}

        const std::string id_;
        const TwistControllerParams& tc_params_;
        const LimiterParams& limiter_params_;
};
/* END ConstraintParamsBase *************************************************************************************/

/* BEGIN ConstraintParamsCA *************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsCA : public ConstraintParamsBase
{
    public:
        ConstraintParamsCA(const TwistControllerParams& params,
                           const LimiterParams& limiter_params,
                           const std::string& id = std::string()) :
                ConstraintParamsBase(params, limiter_params, id)
        {}

        ConstraintParamsCA(const ConstraintParamsCA& cpca) :
                ConstraintParamsBase(cpca.tc_params_, cpca.limiter_params_, cpca.id_)
        {
            current_distances_ = cpca.current_distances_;
        }

        virtual ~ConstraintParamsCA()
        {}

        std::vector<ObstacleDistanceData> current_distances_;
};
/* END ConstraintParamsCA ***************************************************************************************/

/* BEGIN ConstraintParamsJLA ************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsJLA : public ConstraintParamsBase
{
    public:
        ConstraintParamsJLA(const TwistControllerParams& params,
                            const LimiterParams& limiter_params,
                            const std::string& id = std::string()) :
                ConstraintParamsBase(params, limiter_params, id),
                joint_idx_(-1)
        {}

        ConstraintParamsJLA(const ConstraintParamsJLA& cpjla) :
                ConstraintParamsBase(cpjla.tc_params_, cpjla.limiter_params_, cpjla.id_),
                joint_(cpjla.joint_),
                joint_idx_(cpjla.joint_idx_)
        {}

        virtual ~ConstraintParamsJLA()
        {}

        std::string joint_;
        int32_t joint_idx_;
};
/* END ConstraintParamsJLA **************************************************************************************/

typedef boost::shared_ptr<ConstraintParamsBase> ConstraintParamsBase_t;

#endif  // COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_PARAMS_H
