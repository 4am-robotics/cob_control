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
