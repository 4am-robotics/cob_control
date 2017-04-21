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
        ConstraintParamsBase(const ConstraintParams& params,
                             const std::string& id = std::string()) :
                params_(params),
                id_(id)
        {}

        ~ConstraintParamsBase()
        {}

        const std::string id_;
        const ConstraintParams params_;
};
/* END ConstraintParamsBase *************************************************************************************/

/* BEGIN ConstraintParamsCA *************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsCA : public ConstraintParamsBase
{
    public:
        ConstraintParamsCA(const ConstraintParams& params,
                           const std::vector<std::string>& frame_names = std::vector<std::string>(),
                           const std::string& id = std::string()) :
                ConstraintParamsBase(params, id),
                frame_names_(frame_names)
        {}

        ConstraintParamsCA(const ConstraintParamsCA& cpca) :
                ConstraintParamsBase(cpca.params_, cpca.id_),
                frame_names_(cpca.frame_names_),
                current_distances_(cpca.current_distances_)
        {}

        virtual ~ConstraintParamsCA()
        {}

        std::vector<std::string> frame_names_;
        std::vector<ObstacleDistanceData> current_distances_;
};
/* END ConstraintParamsCA ***************************************************************************************/

/* BEGIN ConstraintParamsJLA ************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsJLA : public ConstraintParamsBase
{
    public:
        ConstraintParamsJLA(const ConstraintParams& params,
                            const LimiterParams& limiter_params = LimiterParams(),
                            const std::string& id = std::string()) :
                ConstraintParamsBase(params, id),
                joint_idx_(-1),
                limiter_params_(limiter_params)
        {}

        ConstraintParamsJLA(const ConstraintParamsJLA& cpjla) :
                ConstraintParamsBase(cpjla.params_, cpjla.id_),
                joint_(cpjla.joint_),
                joint_idx_(cpjla.joint_idx_),
                limiter_params_(cpjla.limiter_params_)
        {}

        virtual ~ConstraintParamsJLA()
        {}

        std::string joint_;
        int32_t joint_idx_;
        const LimiterParams& limiter_params_;
};
/* END ConstraintParamsJLA **************************************************************************************/

typedef boost::shared_ptr<ConstraintParamsBase> ConstraintParamsBase_t;

#endif  // COB_TWIST_CONTROLLER_CONSTRAINTS_CONSTRAINT_PARAMS_H
