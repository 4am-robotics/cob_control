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

#ifndef CALLBACK_DATA_MEDIATOR_H_
#define CALLBACK_DATA_MEDIATOR_H_

#include <vector>
#include <stdint.h>
#include <boost/thread/mutex.hpp>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/constraints/constraint_params.h"
#include "cob_obstacle_distance/ObstacleDistances.h"

/// Represents a data pool for distribution of collected data from ROS callback.
class CallbackDataMediator
{
    private:
        std::vector<Distance> obstacle_distances_;
        std::vector<Distance>::const_iterator it_distances;
        boost::mutex distances_to_obstacles_lock_;

    public:
        CallbackDataMediator();

        uint32_t obstacleDistancesCnt();
        bool fill(ConstraintParamsCA& params_ca);
        bool fill(ConstraintParamsJLA& params_jla);
        void distancesToObstaclesCallback(const cob_obstacle_distance::ObstacleDistances::ConstPtr& msg);
};

#endif /* CALLBACK_DATA_MEDIATOR_H_ */
