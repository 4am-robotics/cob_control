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


//void ShapesManager::addShape(Ptr_IMarkerShape_t s)
//{
//    if(this->shapes_.size() <= 0)
//    {
//        this->shapes_.push_back(s);
//    }
//    else
//    {
//        bool already_existent = false;
//        for(std::vector<Ptr_IMarkerShape_t>::const_iterator it = this->shapes_.begin(); it != this->shapes_.end(); it++)
//        {
//            if(s->getId() == (*it)->getId())
//            {
//                already_existent = true;
//                break;
//            }
//        }
//
//        if(!already_existent)
//        {
//            this->shapes_.push_back(s);
//        }
//    }
//}

void ShapesManager::addShape(const std::string& id, Ptr_IMarkerShape_t s)
{
    this->shapes_2_[id] = s;
}

bool ShapesManager::getShape(const std::string& id, Ptr_IMarkerShape_t& s)
{
    bool success = false;
    if(this->shapes_2_.count(id))
    {
        s = this->shapes_2_[id];
        success = true;
    }

    return success;
}


void ShapesManager::draw(bool enforce_draw)
{
//    for(Iter_t iter = shapes_.begin(); iter != shapes_.end(); ++iter)
//    {
//        if(!((*iter)->isDrawn()) || enforce_draw)
//        {
//            ROS_INFO_STREAM("Publishing marker #" << (*iter)->getId() << std::endl);
//            visualization_msgs::Marker marker = (*iter)->getMarker();
//            this->pub_.publish(marker);
//            (*iter)->setDrawn();
//            sleep(1.5); // it takes some time for Rviz to compute and show the marker!
//        }
//    }

    for(Map_iter_t iter = shapes_2_.begin(); iter != shapes_2_.end(); ++iter)
    {
        Ptr_IMarkerShape_t elem = iter->second;
        if(!(elem->isDrawn()) || enforce_draw)
        {
            ROS_INFO_STREAM("Publishing marker #" << elem->getId() << std::endl);
            visualization_msgs::Marker marker = elem->getMarker();
            this->pub_.publish(marker);
            elem->setDrawn();
            sleep(1.5); // it takes some time for Rviz to compute and show the marker!
        }
    }
}


void ShapesManager::clear()
{
    this->shapes_2_.clear();
}
