/*
 * callback_data_mediator.h
 *
 *  Created on: Jun 8, 2015
 *      Author: fxm-mb
 */

#ifndef CALLBACK_DATA_MEDIATOR_H_
#define CALLBACK_DATA_MEDIATOR_H_

#include <vector>
#include <Eigen/Core>
#include <stdint.h>
#include <boost/thread/mutex.hpp>

#include "cob_twist_controller/constraints/constraint_params.h"
#include "cob_obstacle_distance/ObstacleDistances.h"



class CallbackDataMediator
{
    private:
        std::vector<Distance> obstacle_distances_;
        boost::mutex distances_to_obstacles_lock_;
        int test;

    public:
        CallbackDataMediator();

        uint32_t obstacleDistancesCnt();

        bool fill(boost::shared_ptr<ConstraintParamsCA> params_ca);
        bool fill(boost::shared_ptr<ConstraintParamsJLA> params_jla);

        void distancesToObstaclesCallback(const cob_obstacle_distance::ObstacleDistances::ConstPtr& msg);




};

#endif /* CALLBACK_DATA_MEDIATOR_H_ */
