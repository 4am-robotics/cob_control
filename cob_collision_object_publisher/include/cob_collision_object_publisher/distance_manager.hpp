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
 *   ROS package name: cob_collision_object_publisher
 *
 * \author
 *   Author: Marco Bezzon, email: Marco.Bezzon@ipa.fraunhofer.de
 *
 * \date Date of creation: May, 2015
 *
 * \brief
 *   This header contains a definition of a distance calculation management class.
 *   It provides space for obstacles shapes and object of interest shapes in extra shape management objects.
 *   Provides methods for remote calls.
 ****************************************************************/

#ifndef DISTANCE_MANAGER_HPP_
#define DISTANCE_MANAGER_HPP_

#include <boost/scoped_ptr.hpp>
#include "cob_collision_object_publisher/ObjectOfInterest.h"
#include "cob_collision_object_publisher/marker_shapes.hpp"
#include "cob_collision_object_publisher/shapes_manager.hpp"

class DistanceManager
{
    private:
        boost::scoped_ptr<ShapesManager> obstacleMgr_;
        boost::scoped_ptr<ShapesManager> objectOfInterestMgr_;
        const ros::Publisher& pub_;
        const ros::Publisher& distPub_;

    public:

        DistanceManager(const ros::Publisher& distPub, const ros::Publisher& pub);
        ~DistanceManager();

        void clear();
        void addObstacle(tPtrMarkerShapeBase s);
        void addObjectOfInterest(tPtrMarkerShapeBase s);
        void drawObstacles(bool enforceDraw = false);
        void drawObjectsOfInterest(bool enforceDraw = false);
        bool collide(tPtrMarkerShapeBase s1, tPtrMarkerShapeBase s2);
        bool getSmallestDistance(cob_collision_object_publisher::ObjectOfInterest::Request& request,
                                 cob_collision_object_publisher::ObjectOfInterest::Response& response);
};





#endif /* DISTANCE_MANAGER_HPP_ */
