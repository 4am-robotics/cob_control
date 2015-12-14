/*!
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
 *   ROS package name: cob_cartesian_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: July, 2015
 *
 * \brief
 *   ...
 *
 ****************************************************************/

#include <ros/ros.h>
#include <cob_cartesian_controller/cartesian_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_controller_node");
    CartesianController cc;

    if (!cc.initialize())
    {
        ROS_ERROR("Initialization failed");
        return -1;
    }

    ros::spin();
}
