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
 *   Implementation of the MarkerShape interface.
 *   Important to note: increases a static class counter to ensure
 *   a unique ID for the rviz Marker publishing.
 ****************************************************************/
#include "cob_obstacle_distance/marker_shapes/marker_shapes_interface.hpp"

/* BEGIN IMarkerShape *******************************************************************************************/
/// Interface class marking methods that have to be implemented in derived classes.
IMarkerShape::IMarkerShape()
{
    class_ctr_++;
}

uint32_t IMarkerShape::class_ctr_ = 0;
/* END IMarkerShape *********************************************************************************************/
