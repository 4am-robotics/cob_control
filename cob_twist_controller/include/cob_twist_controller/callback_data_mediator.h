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
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: June, 2015
 *
 * \brief
 *   CallbackDataMediator class declaration.
 *   Collects data from ROS callbacks and provides the data
 *   for constraint parameters.
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_CALLBACK_DATA_MEDIATOR_H
#define COB_TWIST_CONTROLLER_CALLBACK_DATA_MEDIATOR_H

#include <vector>
#include <stdint.h>
#include <boost/thread/mutex.hpp>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraints/constraint_params.h"
#include "cob_control_msgs/ObstacleDistances.h"

/// Represents a data pool for distribution of collected data from ROS callback.
class CallbackDataMediator
{
    private:
        typedef ObstacleDistancesInfo_t::const_iterator ObstacleDistancesIter_t;
        ObstacleDistancesInfo_t obstacle_distances_;
        boost::mutex distances_to_obstacles_lock_;

    public:
        CallbackDataMediator() {}

        /**
         * @return Number of active distances to obstacles.
         */
        uint32_t obstacleDistancesCnt();

        /**
         * Special implementation for Collision Avoidance parameters.
         * @param params_ca Reference to Collision Avoidance parameters.
         * @return Success of filling parameters.
         */
        bool fill(ConstraintParamsCA& params_ca);

        /**
         * Special implementation for Joint Limit Avoidance parameters.
         * @param params_jla Reference to JLA parameters.
         * @return Success of filling parameters.
         */
        bool fill(ConstraintParamsJLA& params_jla);

        /**
         * Callback method that can be used by a ROS subscriber to a obstacle distance topic.
         * @param msg The published message containting obstacle distances.
         */
        void distancesToObstaclesCallback(const cob_control_msgs::ObstacleDistances::ConstPtr& msg);
};

#endif  // COB_TWIST_CONTROLLER_CALLBACK_DATA_MEDIATOR_H
