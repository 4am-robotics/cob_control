/*
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
 *   ROS package name: cob_obstacle_distance
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: May, 2015
 *
 * \brief
 *   Implementation of the ShapesManager definitions.
 ****************************************************************/
#include "cob_obstacle_distance/shapes_manager.hpp"


ShapesManager::ShapesManager(const ros::Publisher &pub) : pub_(pub)
{
}


ShapesManager::~ShapesManager()
{
    this->clear();
}


void ShapesManager::addShape(Ptr_IMarkerShape_t s)
{
    this->shapes_.push_back(s);
}


void ShapesManager::draw(bool enforce_draw)
{
    for(Iter_t iter = shapes_.begin(); iter != shapes_.end(); ++iter)
    {
        if(!((*iter)->isDrawn()) || enforce_draw)
        {
            ROS_INFO_STREAM("Publishing marker #" << (*iter)->getId() << std::endl);
            visualization_msgs::Marker marker = (*iter)->getMarker();
            this->pub_.publish(marker);
            (*iter)->setDrawn();
            sleep(1.5); // it takes some time for Rviz to compute and show the marker!
        }
    }
}


void ShapesManager::clear()
{
    this->shapes_.clear();
}
