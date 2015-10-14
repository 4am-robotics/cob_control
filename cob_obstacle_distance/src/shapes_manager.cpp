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

ShapesManager::ShapesManager(const ros::Publisher& pub) : pub_(pub)
{
}


ShapesManager::~ShapesManager()
{
    this->clear();
}


void ShapesManager::addShape(const std::string& id, PtrIMarkerShape_t s)
{
    this->shapes_[id] = s;
}


void ShapesManager::removeShape(const std::string& id)
{
    if(this->shapes_.count(id))
    {
        PtrIMarkerShape_t s = this->shapes_[id];
        visualization_msgs::Marker marker = s->getMarker();
        marker.action = visualization_msgs::Marker::DELETE;
        this->pub_.publish(marker);
    }

    this->shapes_.erase(id);
}


bool ShapesManager::getShape(const std::string& id, PtrIMarkerShape_t& s)
{
    bool success = false;
    if(this->shapes_.count(id))
    {
        s = this->shapes_[id];
        success = true;
    }

    return success;
}


void ShapesManager::draw()
{
    visualization_msgs::MarkerArray marker_array;
    for(MapIter_t iter = shapes_.begin(); iter != shapes_.end(); ++iter)
    {
        PtrIMarkerShape_t elem = iter->second;
        if(elem->isDrawable())
        {
            visualization_msgs::Marker marker = elem->getMarker();
            marker_array.markers.push_back(marker);
        }
    }

    this->pub_.publish(marker_array);
    sleep(0.1); // it takes some time for Rviz to compute and show the marker!
}


void ShapesManager::clear()
{
    this->shapes_.clear();
}


uint32_t ShapesManager::count() const
{
    return this->shapes_.size();
}


uint32_t ShapesManager::count(const std::string& id) const
{
    return this->shapes_.count(id);
}
