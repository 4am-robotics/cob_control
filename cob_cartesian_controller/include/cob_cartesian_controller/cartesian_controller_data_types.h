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


#ifndef COB_CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_DATA_TYPES_H
#define COB_CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_DATA_TYPES_H

#include <vector>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>

namespace cob_cartesian_controller
{

struct ProfileStruct
{
    double t_ipo;
    unsigned int profile_type;
    double vel, accl;
    double Se_max;
};

struct ProfileTimings
{
    double tb, te, tv;
    unsigned int steps_tb, steps_te, steps_tv;
    double vel;
};

struct MoveLinStruct
{
    geometry_msgs::Pose start, end;
};

struct MoveCircStruct
{
    geometry_msgs::Pose pose_center;
    double start_angle, end_angle;
    double radius;
};

struct CartesianActionStruct
{
    unsigned int move_type;
    MoveLinStruct move_lin;
    MoveCircStruct move_circ;
    ProfileStruct profile;
};

class PathArray
{
    public:
        PathArray(const double Se, const std::vector<double> array)
        :    Se_(Se),
             array_(array)
        {}

        ~PathArray()
        {
            array_.clear();
        }

        double Se_;
        std::vector<double> array_;
};

class PathMatrix
{
    public:
        PathMatrix(const PathArray pa1,
                   const PathArray pa2)
        {
            pm_.push_back(pa1);
            pm_.push_back(pa2);
        }

        ~PathMatrix()
        {
            pm_.clear();
        }

        double getMaxSe()
        {
            double se_max = 0;

            for (unsigned int i = 0; i < pm_.size(); i++)
            {
                if (se_max < fabs(pm_[i].Se_))
                {
                    se_max = fabs(pm_[i].Se_);
                }
            }
            return se_max;
        }

        std::vector<PathArray> pm_;
};

}  // namespace cob_cartesian_controller

#endif  // COB_CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_DATA_TYPES_H
