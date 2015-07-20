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
#include <thread>

#include "cob_obstacle_distance/distance_manager.hpp"
#include "cob_obstacle_distance/marker_shapes/marker_shapes.hpp"


void addTestObstacles(DistanceManager& dm);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cob_obstacle_distance");
    ros::NodeHandle nh;
    ros::Rate r(1.0);
    DistanceManager sm(nh);

    if (0 != sm.init())
    {
        ROS_ERROR("Failed to initialize DistanceManager.");
        return -4;
    }

    ros::Subscriber jointstate_sub = nh.subscribe("joint_states", 1, &DistanceManager::jointstateCb, &sm);
    ros::Subscriber obstacle_sub = nh.subscribe("obstacle_distance/registerObstacle", 1, &DistanceManager::registerObstacle, &sm);
    ros::ServiceServer registration_srv = nh.advertiseService("obstacle_distance/registerPointOfInterest" , &DistanceManager::registerPointOfInterest, &sm);
    ros::ServiceServer distance_prediction_srv = nh.advertiseService("obstacle_distance/predictDistance" , &DistanceManager::predictDistance, &sm);

    //addTestObstacles(sm); // Comment in to see what happens

    std::thread t(&DistanceManager::transform, std::ref(sm));
    ROS_INFO_STREAM("Started transform thread.");

    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        sm.calculate();
        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}


/**
 * Dummy publisher for some example fcl <-> rviz markers.
 * @param dm The distance manager reference
 */
void addTestObstacles(DistanceManager& dm)
{
    fcl::Sphere s(0.1);
    fcl::Box b(0.1, 0.1, 0.1); // Take care the nearest point for collision is one of the eight corners!!! This might lead to jittering

    PtrIMarkerShape_t sptr_Bvh(new MarkerShape<BVH_RSS_t>(dm.getRootFrame(),
                                                          "package://cob_gazebo_objects/Media/models/milk.dae",
                                                           -0.35,
                                                           -0.35,
                                                            0.8));


    PtrIMarkerShape_t sptr_Sphere(new MarkerShape<fcl::Sphere>(dm.getRootFrame(), s, 0.35, -0.35, 0.8));

    dm.addObstacle("Funny Sphere", sptr_Sphere);
    dm.addObstacle("Funny Mesh", sptr_Bvh);

    ROS_INFO_ONCE("Subscriber to the marker has been created");
    dm.drawObstacles();

}
