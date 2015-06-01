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
 *   Implementation of the DistanceManager definitions.
 ****************************************************************/

#include "cob_collision_object_publisher/distance_manager.hpp"

#include "fcl/collision_object.h"
#include "fcl/collision.h"
#include "fcl/distance.h"
#include "fcl/collision_data.h"

#include <std_msgs/Float64.h>

DistanceManager::DistanceManager(const ros::Publisher &distPub, const ros::Publisher &pub) : distPub_(distPub), pub_(pub)
{
    this->obstacleMgr_.reset(new ShapesManager(pub));
    this->objectOfInterestMgr_.reset(new ShapesManager(pub));
}

DistanceManager::~DistanceManager()
{
}

void DistanceManager::clear()
{
    this->obstacleMgr_->clear();
    this->objectOfInterestMgr_->clear();
}

void DistanceManager::addObstacle(tPtrMarkerShapeBase s)
{
    this->obstacleMgr_->addShape(s);
}

void DistanceManager::addObjectOfInterest(tPtrMarkerShapeBase s)
{
    this->objectOfInterestMgr_->addShape(s);
}

void DistanceManager::drawObstacles(bool enforceDraw)
{
    this->obstacleMgr_->draw(enforceDraw);
}

void DistanceManager::drawObjectsOfInterest(bool enforceDraw)
{
    this->objectOfInterestMgr_->draw(enforceDraw);
}

bool DistanceManager::collide(tPtrMarkerShapeBase s1, tPtrMarkerShapeBase s2)
{
    fcl::CollisionObject x = s1->getCollisionObject();
    fcl::CollisionObject y = s2->getCollisionObject();
    fcl::CollisionResult result;
    fcl::CollisionRequest request(1, true);

    fcl::collide(&x, &y, request, result);

    ROS_INFO_STREAM("isCollision: " << result.isCollision() << std::endl);

    fcl::DistanceResult resultD;
    fcl::DistanceRequest requestD(true, 1.0, 100.0);


    const clock_t begin_time = clock();
    fcl::FCL_REAL dist = fcl::distance(&x, &y, requestD, resultD);
    ROS_INFO_STREAM("Duration of distance calc: " << float( clock () - begin_time ) /  CLOCKS_PER_SEC);

    ROS_INFO_STREAM("Min distance: " << dist << std::endl);
    ROS_INFO_STREAM("Nearest Point 1 (rel. from the center of object 1): " << resultD.nearest_points[0] << std::endl);
    ROS_INFO_STREAM("Nearest Point 1 (abs. from base link): " << (resultD.nearest_points[0] + x.getTranslation()) << std::endl);
    ROS_INFO_STREAM("Nearest Point 2 (from the center of object 2(real coordinate is pos + vec)): " << resultD.nearest_points[1] << std::endl);
    ROS_INFO_STREAM("Nearest Point 2 (abs. from base link): " << (resultD.nearest_points[1] + y.getTranslation()) << std::endl);

//            fcl::DistanceResult resultD2;
//            fcl::DistanceRequest requestD2(true, 10.0, 1.0);
//            const clock_t begin_time2 = clock();
//            fcl::FCL_REAL dist2 = fcl::distance(&x, &y, requestD2, resultD2);
//            ROS_INFO_STREAM("Duration of distance calc (with errors activated): " << float( clock () - begin_time2 ) /  CLOCKS_PER_SEC);
//
//            ROS_INFO_STREAM("Min distance: " << dist2 << std::endl);
//            ROS_INFO_STREAM("Nearest Point 1 (rel. from the center of object 1): " << resultD2.nearest_points[0] << std::endl);
//            ROS_INFO_STREAM("Nearest Point 1 (abs. from base link): " << (resultD2.nearest_points[0] + x.getTranslation()) << std::endl);
//            ROS_INFO_STREAM("Nearest Point 2 (from the center of object 2(real coordinate is pos + vec)): " << resultD2.nearest_points[1] << std::endl);
//            ROS_INFO_STREAM("Nearest Point 2 (abs. from base link): " << (resultD2.nearest_points[1] + y.getTranslation()) << std::endl);
}

bool DistanceManager::getSmallestDistance(cob_collision_object_publisher::ObjectOfInterest::Request& request,
                                          cob_collision_object_publisher::ObjectOfInterest::Response& response)
{

    ROS_INFO_STREAM("Retrieved service request!!! ");
    response = cob_collision_object_publisher::ObjectOfInterest::Response();
    bool success = true;
    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 1.0;
    col.g = 0.0;
    col.b = 0.0;

    fcl::Box b(0.1, 0.1, 0.1);
    fcl::Sphere s(0.1);
    fcl::Cylinder c(0.1, 0.1);

    tPtrMarkerShapeBase ooi;
    switch(request.shapeType)
    {
        case visualization_msgs::Marker::CUBE:
            ooi.reset(new MarkerShape<fcl::Box>(b, request.p, col));
            break;
        case visualization_msgs::Marker::SPHERE:
            ooi.reset(new MarkerShape<fcl::Sphere>(s, request.p, col));
            break;
        case visualization_msgs::Marker::CYLINDER:
            ooi.reset(new MarkerShape<fcl::Cylinder>(c, request.p, col));
            break;
        default:
            ROS_ERROR("Failed to process request due to unknown shape type: %d", request.shapeType);
            success = false;
            return false;
    }

//     visualization_msgs::Marker marker = ooi->getMarker();
//    pub_.publish(marker);

    ROS_INFO_STREAM("Trying to get collision object!");
    fcl::CollisionObject ooiCo = ooi->getCollisionObject();

    ooi.reset();

    fcl::DistanceResult distResult;
    bool setDistResult = false;
    //boost::shared_ptr<fcl::CollisionGeometry> sptr;
    fcl::CollisionObject resultOCo = ooiCo;

    fcl::FCL_REAL lastDist = std::numeric_limits<fcl::FCL_REAL>::max();

    ROS_INFO_STREAM("Iteration over obstacles");
    for(ShapesManager::iterator it = this->obstacleMgr_->begin(); it != this->obstacleMgr_->end(); ++it)
    {
        fcl::CollisionObject oCo = (*it)->getCollisionObject();
        fcl::DistanceResult tmpResult;
        fcl::DistanceRequest distRequest(true, 1.0, 100.0);

        ROS_INFO_STREAM("Next calculation of distance: ");

        fcl::FCL_REAL dist = fcl::distance(&ooiCo, &oCo, distRequest, tmpResult);
        ROS_INFO_STREAM("Calculated distance: " << dist);

        if (dist < lastDist)
        {
            setDistResult = true;
            distResult = tmpResult;
            resultOCo = oCo;
        }
    }


    //ROS_INFO_STREAM("Found a dist result? " << setDistResult);
    if(setDistResult)
    {
        //ROS_INFO_STREAM("Min distance: " << distResult.min_distance << std::endl);
        //ROS_INFO_STREAM("Nearest Point 1 (rel. from the center of object 1): " << distResult.nearest_points[0] << std::endl);
        //ROS_INFO_STREAM("Nearest Point 1 (abs. from base link): " << (distResult.nearest_points[0] + ooiCo.getTranslation()) << std::endl);
        //ROS_INFO_STREAM("Nearest Point 2 (from the center of object 2(real coordinate is pos + vec)): " << distResult.nearest_points[1] << std::endl);
        //ROS_INFO_STREAM("Nearest Point 2 (abs. from base link): " << (distResult.nearest_points[1] + resultOCo.getTranslation()) << std::endl);

        ROS_INFO_STREAM("Minimum distance: " << distResult.min_distance);
        response.distance = static_cast<double>(distResult.min_distance);

        fcl::Vec3f t = distResult.nearest_points[1] + resultOCo.getTranslation(); // Translation from "base_link" frame!!!
        fcl::Quaternion3f q = resultOCo.getQuatRotation();

        response.obstacle.orientation.w = q.getW();
        response.obstacle.orientation.x = q.getX();
        response.obstacle.orientation.y = q.getY();
        response.obstacle.orientation.z = q.getZ();

        response.obstacle.position.x = t[0];
        response.obstacle.position.y = t[1];
        response.obstacle.position.z = t[2];
    }

    std_msgs::Float64 f64MinDist;
    f64MinDist.data = static_cast<double>(response.distance);
    this->distPub_.publish(f64MinDist);

//    if (success)
//    {
//        this->addObjectOfInterest(ooi); // TODO: generate id or slot to return? and adapt later because pose changes with time!
//    }


    ROS_INFO_STREAM("Return " << setDistResult);

    return true;
}
