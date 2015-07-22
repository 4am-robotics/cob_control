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

#ifndef COB_CARTESIAN_CONTROLLER_TRAJECTORY_INTERPOLATOR_H_
#define COB_CARTESIAN_CONTROLLER_TRAJECTORY_INTERPOLATOR_H_

#include <ros/ros.h>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator.h>


class TrajectoryInterpolator
{
public:
    TrajectoryInterpolator(double update_rate)
    :   TPG_(TrajectoryProfileGenerator(update_rate))
    {}

    ~TrajectoryInterpolator(){}

    bool linearInterpolation(std::vector <geometry_msgs::Pose>& pose_vector,
                             cob_cartesian_controller::MoveLinStruct& move_lin);

    bool circularInterpolation(std::vector<geometry_msgs::Pose>& pose_vector,
                               cob_cartesian_controller::MoveCircStruct& move_circ);

private:
    TrajectoryProfileGenerator TPG_;
};



#endif /* COB_CARTESIAN_CONTROLLER_TRAJECTORY_INTERPOLATOR_H_ */
