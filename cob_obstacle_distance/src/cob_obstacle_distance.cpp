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
 *   Main to initialize a node and start publishing markers.
 *
 ****************************************************************/
#include <ctime>
#include <vector>
#include <ros/ros.h>
#include <fcl/shape/geometric_shapes.h>
#include <fstream>

#include "cob_obstacle_distance/distance_manager.hpp"
#include "cob_obstacle_distance/marker_shapes.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_obstacle_distance");
    ros::NodeHandle nh("cob_obstacle_distance");
    ros::Rate r(1.0);

    DistanceManager sm(nh);
    if (0 != sm.init())
    {
        ROS_ERROR("Failed to initialize DistanceManager.");
        return -4;
    }

    ros::ServiceServer registration_srv = nh.advertiseService(sm.getRoboNamespace() + "/registerPointOfInterest" , &DistanceManager::registerPointOfInterest, &sm);
    ros::Subscriber jointstate_sub = nh.subscribe("/joint_states", 1, &DistanceManager::jointstateCb, &sm);

    ROS_INFO("Starting basic_shapes ...\r\n");
    fcl::Box b(0.3, 0.3, 0.3);
//    tPtrMarkerShapeBase sptr_Cube(new MarkerShape<fcl::Box>(b, 1.0, 1.0, 1.0));
//    tPtrMarkerShapeBase sptr_Cube2(new MarkerShape<fcl::Box>(b, -1.0, -1.0, 1.0));
//    tPtrMarkerShapeBase sptr_Sphere(new MarkerShape<fcl::Sphere>(1.0, -1.0, -1.0));
//    tPtrMarkerShapeBase sptr_Cyl(new MarkerShape<fcl::Cylinder>(-1.0, 1.0, -1.0));

    tPtrMarkerShapeBase sptr_Cube(new MarkerShape<fcl::Box>(b, 0.5, -0.5, 1.0));
    //tPtrMarkerShapeBase sptr_Cube2(new MarkerShape<fcl::Box>(b, -1.0, -1.0, 1.0));
    //tPtrMarkerShapeBase sptr_Sphere(new MarkerShape<fcl::Sphere>(1.0, -1.0, -1.0));
    //tPtrMarkerShapeBase sptr_Cyl(new MarkerShape<fcl::Cylinder>(-1.0, 1.0, -1.0));

    //IMarkerShape *ims = new XMarkerShapeBase<fcl::Box>(b, 1.0, 1.0, 1.0);
    // XMarkerShapeBase<fcl::Box> testo(b, 1.0, 1.0, 1.0);


    sm.addObstacle(sptr_Cube);
    //sm.addObstacle(sptr_Cube2);
    //sm.addObstacle(sptr_Sphere);
    //sm.addObstacle(sptr_Cyl);
    //sm.addShape(sphere);
    //sm.addShape(cylinder);

    //sm.collide(sptr_Cube, sptr_Cube2);

    if(!sm.waitForMarkerSubscriber())
    {
        return -1;
    }

    ROS_INFO_ONCE("Subscriber to the marker has been created");
    sm.drawObstacles();

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        sm.calculate();
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

