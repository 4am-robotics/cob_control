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
 *   Class implementing the Ramp velocity profile generator.
 *
 ****************************************************************/

#include <vector>
#include <ros/ros.h>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_ramp.h>

/* BEGIN TrajectoryProfileRamp ********************************************************************************************/
inline bool TrajectoryProfileRamp::getProfileTimings(double Se, double te, bool calcMaxTe, cob_cartesian_controller::ProfileTimings& pt)
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

inline std::vector<double> TrajectoryProfileRamp::getTrajectory(double se, cob_cartesian_controller::ProfileTimings pt)
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
