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


#include <vector>
#include <ros/ros.h>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_ramp.h>

/* BEGIN TrajectoryProfileRamp ********************************************************************************************/
bool TrajectoryProfileRamp::getProfileTimings(double Se, double te, bool calcMaxTe, cob_cartesian_controller::ProfileTimings& pt)
{
    CartesianControllerUtils utils;
    double tv, tb = 0.0;
    double vel = params_.profile.vel;
    double accl = params_.profile.accl;

    // Calculate the Sinoid Profile Parameters
    if (vel > sqrt(std::fabs(Se) * accl))
    {
        vel = sqrt(std::fabs(Se) * accl);
    }

    if (vel > MIN_VELOCITY_THRESHOLD)
    {
        tb = utils.roundUpToMultiplier(vel / accl, params_.profile.t_ipo);
        if (calcMaxTe)
        {
            te = utils.roundUpToMultiplier((std::fabs(Se) / vel) + tb,  params_.profile.t_ipo);
        }
        tv = te - tb;

        // Interpolationsteps for every timesequence
        pt.tb = tb;
        pt.tv = tv;
        pt.te = te;

        pt.steps_tb = pt.tb/params_.profile.t_ipo;
        pt.steps_tv = (pt.tv-pt.tb)/params_.profile.t_ipo;
        pt.steps_te = (pt.te-pt.tv)/params_.profile.t_ipo;

        pt.vel = vel;
        return true;
    }

    return false;
}

std::vector<double> TrajectoryProfileRamp::getTrajectory(double se, cob_cartesian_controller::ProfileTimings pt)
{
    std::vector<double> array;
    unsigned int i = 1;
    double direction = se/std::fabs(se);
    double accl = params_.profile.accl;
    double t_ipo = params_.profile.t_ipo;

    // Calculate the ramp profile path
    // 0 <= t <= tb
    for (; i <= pt.steps_tb; i++)
    {
        array.push_back(direction * (0.5*accl*pow((t_ipo*i), 2)));
    }
    // tb <= t <= tv
    for (; i <= (pt.steps_tb + pt.steps_tv); i++)
    {
        array.push_back(direction *(pt.vel*(t_ipo*i) - 0.5*pow(pt.vel, 2)/accl));
    }
    // tv <= t <= te
    for (; i <= (pt.steps_tb + pt.steps_tv + pt.steps_te + 1); i++)
    {
        array.push_back(direction * (pt.vel * pt.tv - 0.5 * accl * pow(pt.te-(t_ipo*i), 2)));
    }

    return array;
}
/* END TrajectoryProfileRamp **********************************************************************************************/
