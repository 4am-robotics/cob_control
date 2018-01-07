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


#ifndef COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_BASE_H
#define COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_BASE_H

#include <vector>
#include <ros/ros.h>
#include <cob_cartesian_controller/cartesian_controller_data_types.h>
#include <cob_cartesian_controller/cartesian_controller_utils.h>


#define MIN_VELOCITY_THRESHOLD 0.001

class TrajectoryProfileBase
{
public:
    explicit TrajectoryProfileBase(const cob_cartesian_controller::CartesianActionStruct& params)
    :    params_(params)
    {}

    virtual ~TrajectoryProfileBase()
    {}

    virtual bool calculateProfile(std::vector<double>* path_matrix,
                                  const double Se_lin, const double Se_rot)
    {
        CartesianControllerUtils ccu;
        std::vector<double> linear_path, angular_path;

        cob_cartesian_controller::PathArray lin(Se_lin, linear_path);
        cob_cartesian_controller::PathArray rot(Se_rot, angular_path);

        cob_cartesian_controller::PathMatrix pm(lin, rot);

        // Get the profile timings from the longest path
        bool success = getProfileTimings(pm.getMaxSe(), 0, true, pt_max_);

        // Calculate the paths
        for (unsigned int i = 0; i < pm.pm_.size(); i++)
        {
            generatePath(pm.pm_[i]);
        }

        // Adjust the array length
        // If no trajectory was interpolated, then this path array contains only one constant value.
        // This constant value needs to be duplicated N_max times for matrix conversion purposes.
        ccu.adjustArrayLength(pm.pm_);

        ccu.copyMatrix(path_matrix, pm.pm_);

        return true;
    }

protected:
    virtual bool generatePath(cob_cartesian_controller::PathArray& pa)
    {
        std::vector<double> array;
        cob_cartesian_controller::ProfileTimings pt;

        // Calculate the Profile Timings
        if (getProfileTimings(pa.Se_, pt_max_.te, false, pt))
        {
            array = getTrajectory(pa.Se_, pt);
        }
        else
        {
            array.push_back(0);
        }

        pa.array_ = array;
        return true;
    }

    virtual bool getProfileTimings(double Se, double te, bool calcMaxTe, cob_cartesian_controller::ProfileTimings& pt) = 0;
    virtual std::vector<double> getTrajectory(double se, cob_cartesian_controller::ProfileTimings pt) = 0;

    const cob_cartesian_controller::CartesianActionStruct& params_;
    cob_cartesian_controller::ProfileTimings pt_max_;
};

#endif  // COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_BASE_H
