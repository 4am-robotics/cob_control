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


#include <ctime>
#include <vector>
#include <ros/ros.h>
#include <fstream>
#include <thread>

#include <fcl/config.h>
#if FCL_MINOR_VERSION == 5
    #include <fcl/shape/geometric_shapes.h>
    typedef fcl::Box FCL_Box;
    typedef fcl::Sphere FCL_Sphere;
    typedef fcl::Cylinder FCL_Cylinder;
#else
    #include <fcl/geometry/shape/box.h>
    #include <fcl/geometry/shape/sphere.h>
    #include <fcl/geometry/shape/cylinder.h>
    typedef fcl::Boxf FCL_Box;
    typedef fcl::Spheref FCL_Sphere;
    typedef fcl::Cylinderf FCL_Cylinder;
#endif


#include "cob_obstacle_distance/obstacle_distance_data_types.hpp"
#include "cob_obstacle_distance/distance_manager.hpp"
#include "cob_obstacle_distance/marker_shapes/marker_shapes.hpp"


void addTestObstacles(DistanceManager& dm);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cob_obstacle_distance");
    ros::NodeHandle nh;
    DistanceManager sm(nh);

    if (0 != sm.init())
    {
        ROS_ERROR("Failed to initialize DistanceManager.");
        return -4;
    }

    // addTestObstacles(sm);  // Comment in to see what happens

    std::thread t(&DistanceManager::transform, std::ref(sm));
    ros::Duration(1.0).sleep();  // Give some time for threads to run
    ROS_INFO_STREAM("Started transform thread.");

    ros::Subscriber jointstate_sub = nh.subscribe("joint_states", 1, &DistanceManager::jointstateCb, &sm);
    ros::Subscriber obstacle_sub = nh.subscribe("obstacle_distance/registerObstacle", 1, &DistanceManager::registerObstacle, &sm);
    ros::ServiceServer registration_srv = nh.advertiseService("obstacle_distance/registerLinkOfInterest" , &DistanceManager::registerLinkOfInterest, &sm);

    ros::Rate loop_rate(20);
    while (ros::ok())
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
    FCL_Sphere s(0.1);
    FCL_Box b(0.1, 0.1, 0.1);  // Take care the nearest point for collision is one of the eight corners!!! This might lead to jittering

    PtrIMarkerShape_t sptr_Bvh(new MarkerShape<BVH_RSS_t>(dm.getRootFrame(),
                                                          "package://cob_gazebo_objects/Media/models/milk.dae",
                                                           -0.35,
                                                           -0.35,
                                                            0.8));

    PtrIMarkerShape_t sptr_Sphere(new MarkerShape<FCL_Sphere>(dm.getRootFrame(), s, 0.35, -0.35, 0.8));

    dm.addObstacle("Funny Sphere", sptr_Sphere);
    dm.addObstacle("Funny Mesh", sptr_Bvh);

    ROS_INFO_ONCE("Subscriber to the marker has been created");
    dm.drawObstacles();
}
