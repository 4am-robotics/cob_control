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
 * \date Date of creation: December, 2015
 *
 * \brief
 *   This class contains the implementation for the linear and circular interpolation.
 *
 ****************************************************************/

#ifndef COB_CARTESIAN_CONTROLLER_TRAJECTORY_INTERPOLATOR_TRAJECTORY_INTERPOLATOR_H
#define COB_CARTESIAN_CONTROLLER_TRAJECTORY_INTERPOLATOR_TRAJECTORY_INTERPOLATOR_H

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

#include <cob_cartesian_controller/cartesian_controller_data_types.h>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_builder.h>

class TrajectoryInterpolator
{
public:
    TrajectoryInterpolator(std::string root_frame, double update_rate)
    :   root_frame_(root_frame)
    {}

    ~TrajectoryInterpolator()
    {
        trajectory_profile_generator_.reset();
    }

    bool linearInterpolation(geometry_msgs::PoseArray& pose_array,
                             const cob_cartesian_controller::CartesianActionStruct as);

    bool circularInterpolation(geometry_msgs::PoseArray& pose_array,
                               const cob_cartesian_controller::CartesianActionStruct as);

private:
    std::string root_frame_;
    boost::shared_ptr<TrajectoryProfileBase> trajectory_profile_generator_;
};

#endif  // COB_CARTESIAN_CONTROLLER_TRAJECTORY_INTERPOLATOR_TRAJECTORY_INTERPOLATOR_H
