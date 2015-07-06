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


#include <assimp/scene.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_obstacle_distance");
    ros::NodeHandle nh;
    ros::Rate r(1.0);
    DistanceManager sm(nh);

    aiNode ainode;

    if (0 != sm.init())
    {
        ROS_ERROR("Failed to initialize DistanceManager.");
        return -4;
    }

    ros::Subscriber jointstate_sub = nh.subscribe("joint_states", 1, &DistanceManager::jointstateCb, &sm);
    ros::ServiceServer registration_srv = nh.advertiseService("obstacle_distance/registerPointOfInterest" , &DistanceManager::registerPointOfInterest, &sm);
    ros::ServiceServer distance_prediction_srv = nh.advertiseService("obstacle_distance/predictDistance" , &DistanceManager::predictDistance, &sm);

    ROS_INFO("Starting basic_shapes ...\r\n");
    fcl::Sphere s(0.1);
    fcl::Box b(0.1, 0.1, 0.1); // Take care the nearest point for collision is one of the eight corners!!! This might lead to jittering

    Ptr_IMarkerShape_t sptr_Bvh(new MarkerShape<BVH_RSS_t>(sm.getRootFrame(),
                                                           "package://schunk_description/meshes/lwa4p_extended/arm_1_collision.stl",
                                                           -0.35,
                                                           -0.35,
                                                           0.8));


    Ptr_IMarkerShape_t sptr_Sphere(new MarkerShape<fcl::Sphere>(sm.getRootFrame(), s, 0.35, -0.35, 0.8));

    //Ptr_IMarkerShape_t sptr_Cube(new MarkerShape<fcl::Box>(b, 0.35, -0.35, 0.8));
    // Ptr_IMarkerShape_t sptr_Sphere(new MarkerShape<fcl::Sphere>(1.0, -1.0, -1.0));
    // Ptr_IMarkerShape_t sptr_Cyl(new MarkerShape<fcl::Cylinder>(-1.0, 1.0, -1.0));

    sm.addObstacle(sptr_Sphere);
    sm.addObstacle(sptr_Bvh);
    // sm.addObstacle(sptr_Cube);
    // sm.addObstacle(sptr_Cyl);

    ROS_INFO_ONCE("Subscriber to the marker has been created");
    sm.drawObstacles();

    ros::Rate loop_rate(60);
    while(ros::ok())
    {
        sm.calculate();
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

