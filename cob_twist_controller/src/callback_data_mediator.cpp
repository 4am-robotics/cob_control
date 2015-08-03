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
 *   Methods implementation of the callback data mediator.
 *   Collects data from ROS callbacks and provides the data
 *   for constraint parameters.
 *
 ****************************************************************/
#include <ros/ros.h>

#include "cob_twist_controller/callback_data_mediator.h"

#include <eigen_conversions/eigen_msg.h>

CallbackDataMediator::CallbackDataMediator()
{
    this->it_distances = this->obstacle_distances_.end();
}

/// Counts all currently available distances to obstacles.
uint32_t CallbackDataMediator::obstacleDistancesCnt()
{
    boost::mutex::scoped_lock lock(distances_to_obstacles_lock_);
    return this->obstacle_distances_.size();
}

/// Consumer: Consumes elements from distances container
bool CallbackDataMediator::fill(ConstraintParamsCA& params_ca)
{
    boost::mutex::scoped_lock lock(distances_to_obstacles_lock_);
    bool success = false;
    if (this->obstacle_distances_.end() != this->it_distances)
    {
        params_ca.current_distance_.distance_vec = this->it_distances->distance_vec;
        params_ca.current_distance_.frame_id = this->it_distances->frame_id;
        params_ca.current_distance_.min_distance = this->it_distances->min_distance;
        params_ca.current_distance_.collision_pnt_vector = this->it_distances->collision_pnt_vector;
        this->it_distances++;

        // Let the iterator point to the first element again -> Returns the same elements again until callback occurred.
        if (this->obstacle_distances_.end() == this->it_distances)
        {
            this->it_distances = this->obstacle_distances_.begin();
        }

        success = true;
    }

    return success;
}

/// Can be used to fill parameters for joint limit avoidance.
bool CallbackDataMediator::fill(ConstraintParamsJLA& params_jla)
{
    return true;
}

/// Producer: Fills obstacle distances but only when they are empty
void CallbackDataMediator::distancesToObstaclesCallback(const cob_obstacle_distance::ObstacleDistances::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(distances_to_obstacles_lock_);
    this->obstacle_distances_.clear();
    for(cob_obstacle_distance::ObstacleDistances::_distances_type::const_iterator it = msg->distances.begin(); it != msg->distances.end(); it++)
    {
        ObstacleDistanceInfo d;
        d.min_distance = it->distance;

        d.distance_vec << it->distance_vector.x,
                          it->distance_vector.y,
                          it->distance_vector.z;
        tf::vectorMsgToEigen(it->collision_pnt_vector, d.collision_pnt_vector);

        d.frame_id = it->header.frame_id;
        this->obstacle_distances_.push_back(d);
    }

    this->it_distances = this->obstacle_distances_.begin();
}
