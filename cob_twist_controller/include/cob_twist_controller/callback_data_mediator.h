/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


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
