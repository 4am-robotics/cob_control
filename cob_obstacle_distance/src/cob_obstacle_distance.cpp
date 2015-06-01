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
#include <std_msgs/Float64.h>

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
    ros::NodeHandle nh;
    ros::Rate r(1.0);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher distance_pub = nh.advertise<std_msgs::Float64>("minimal_distance", 1);


    ROS_INFO("Starting basic_shapes ...\r\n");


    fcl::Box b(0.3, 0.3, 0.3);

    DistanceManager sm(distance_pub, marker_pub);

    ros::ServiceServer service = nh.advertiseService("getSmallestDistance", &DistanceManager::getSmallestDistance, &sm);

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

    while (ros::ok() && nh.ok())
    {
      if(!waitForSubscriber(marker_pub))
      {
          return -1;
      }

      ROS_INFO_ONCE("Subscriber to the marker has been created");
      sm.drawObstacles();

      ros::spinOnce();
    }


    ROS_INFO_ONCE("Clean up!!!");
    sm.clear();

    return 0;
}

