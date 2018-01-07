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


#include <string>
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
    if (this->shapes_.count(id))
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
    if (this->shapes_.count(id))
    {
        s = this->shapes_[id];
        success = true;
    }

    return success;
}


void ShapesManager::draw()
{
    visualization_msgs::MarkerArray marker_array;
    for (MapIter_t iter = shapes_.begin(); iter != shapes_.end(); ++iter)
    {
        PtrIMarkerShape_t elem = iter->second;
        if(elem->isDrawable())
        {
            visualization_msgs::Marker marker = elem->getMarker();
            marker_array.markers.push_back(marker);
        }
    }

    this->pub_.publish(marker_array);
    sleep(0.1);  // it takes some time for Rviz to compute and show the marker!
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
