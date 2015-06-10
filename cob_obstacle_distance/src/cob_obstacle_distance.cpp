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
#include <visualization_msgs/Marker.h>
#include <fstream>

#include "cob_obstacle_distance/distance_manager.hpp"
#include "cob_obstacle_distance/marker_shapes.hpp"

bool waitForSubscriber(ros::Publisher &pub)
{
    while (pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return false;
      }

      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_obstacle_distance");
    ros::NodeHandle nh("cob_obstacle_distance");
    ros::Rate r(1.0);

    std::string robo_namespace = "arm_right"; // replace with nh.getNamespace()

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher obstacle_dist_pub = nh.advertise<cob_obstacle_distance::ObstacleDistances>(robo_namespace, 1);

    ROS_INFO_STREAM("Topic_name: " << obstacle_dist_pub.getTopic());

    ROS_INFO("Starting basic_shapes ...\r\n");



    //KDL::Frame pOut;
    //int retVal = adChnFkSolverPos.JntToCart(q_in, pOut);

    // KDL::Frame joint4pos = adChnFkSolverPos.getPostureAtJnt(3);


    fcl::Box b(0.3, 0.3, 0.3);

    DistanceManager sm(marker_pub, obstacle_dist_pub, nh);
    if (0 != sm.init())
    {
        ROS_ERROR("Failed to initialize DistanceManager.");
        return -1;
    }

    //ros::ServiceServer service = nh.advertiseService("getSmallestDistance", &DistanceManager::getSmallestDistance, &sm);
    //ros::ServiceServer service2 = nh.advertiseService("getAnotherSmallestDistance", &DistanceManager::getAnotherSmallestDistance, &sm);
    ros::ServiceServer service3 = nh.advertiseService(robo_namespace + "/registerPointOfInterest" , &DistanceManager::registerPointOfInterest, &sm);
    ros::Subscriber jointstate_sub = nh.subscribe("/joint_states", 1, &DistanceManager::jointstateCb, &sm);



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

    if(!waitForSubscriber(marker_pub))
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

        //ROS_INFO("loop");
    }


    return 0;
}

