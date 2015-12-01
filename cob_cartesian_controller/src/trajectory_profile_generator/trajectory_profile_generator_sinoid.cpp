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
 * \date Date of creation: September, 2015
 *
 * \brief
 *
 ****************************************************************/
#include "ros/ros.h"
#include "cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_sinoid.h"

/* BEGIN TrajectoryProfileSinoid ****************************************************************************************/
inline cob_cartesian_controller::ProfileTimings TrajectoryProfileSinoid::getProfileTimings(double Se, double te, double accl, double vel, bool calcMaxTe)
{
    CartesianControllerUtils utils;
    cob_cartesian_controller::ProfileTimings pt;
    int steps_te, steps_tv, steps_tb = 0;
    double tv, tb = 0.0;

    // Calculate the Sinoid Profile Parameters
    if (vel > sqrt(std::fabs(Se) * accl / 2))
    {
        vel = sqrt(std::fabs(Se) * accl / 2);
    }

    if(vel > 0.001)
    {
        tb = utils.roundUpToMultiplier(2 * vel / accl, params_.profile.t_ipo);

        if(te == 0)
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

        ROS_INFO_STREAM("=========================================================================");
        ROS_INFO_STREAM("vel_new:" << vel << " accl: " << accl << " Se: " << std::fabs(Se));
        ROS_INFO_STREAM("pt.tb:" << pt.tb << " pt.tv: " << pt.tv << " pt.te: " << pt.te);
        ROS_INFO_STREAM("pt.steps_tb:" << pt.steps_tb << " pt.steps_tv: " << pt.steps_tv << " pt.steps_te: " << pt.steps_te);
        ROS_INFO_STREAM("=========================================================================");

        pt.ok = true;
    }
    else
    {
        pt.ok = false;
    }

    return pt;
}


inline bool TrajectoryProfileSinoid::generatePath(cob_cartesian_controller::PathArray &pa)
{
    std::vector<double> array;
    cob_cartesian_controller::ProfileTimings pt;
    double accl_max = params_.profile.accl;
    double vel_max = params_.profile.vel;

    // Calculate the Profile Timings
    pt = getProfileTimings(pa.Se_, pt_max_.te, accl_max, vel_max, false);
    if(pt.ok)
    {
        array = getTrajectory(pa.Se_, accl_max, pt.vel, params_.profile.t_ipo, pt.steps_tb, pt.steps_tv, pt.steps_te, pt.tb, pt.tv, pt.te);
    }
    else
    {
        array.push_back(0);
    }

    pa.array_ = array;
    return true;
}

inline std::vector<double> TrajectoryProfileSinoid::getTrajectory(double se, double accl, double vel, double t_ipo, double steps_tb, double steps_tv, double steps_te, double tb, double tv, double te)
{
    std::vector<double> array;
    unsigned int i = 1;
    double direction = se/std::fabs(se);

    //Calculate the sinoid profile path
    // 0 <= t <= tb
    for(; i <= steps_tb; i++)
    {
        array.push_back(direction * (accl*(0.25*pow(i*t_ipo,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*t_ipo))-1))));
    }
    // tb <= t <= tv
    for(; i <= (steps_tb + steps_tv); i++)
    {
        array.push_back(direction * ( vel*(i*t_ipo-0.5*tb)));
    }
    // tv <= t <= te
    for(; i <= (steps_tv + steps_tb + steps_te + 1); i++)
    {
        array.push_back(direction * (0.5 * accl *(te*(i*t_ipo + tb) - 0.5*(pow(i*t_ipo,2)+pow(te,2)+2*pow(tb,2)) + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos(((2*M_PI)/tb) * (i*t_ipo-tv))))));
    }

    return array;
}

inline bool TrajectoryProfileSinoid::calculateProfile(std::vector<double> path_matrix[2],
                                                    const double Se_lin, const double Se_rot,
                                                    geometry_msgs::Pose start)
{
    CartesianControllerUtils ccu;
    std::vector<double> linear_path, angular_path;

    cob_cartesian_controller::PathArray lin(Se_lin, linear_path);
    cob_cartesian_controller::PathArray rot(Se_rot, angular_path);


    cob_cartesian_controller::PathMatrix pm(lin,rot);

    // Get the profile timings from the longest path
    pt_max_ = getProfileTimings(pm.getMaxSe(), 0, params_.profile.accl, params_.profile.vel, true);

    // Calculate the paths
    for(int i=0; i<pm.pm_.size(); i++)
    {
        generatePath(pm.pm_[i]);
    }

    // Adjust the array length
    // If no trajectory was interpolated, then this path array contains only one constant value.
    // This constant value needs to be duplicated N_max times for matrix conversion purposes.
    ccu.adjustArrayLength(pm.pm_);

    ccu.copyMatrix(path_matrix,pm.pm_);

    return true;
}
/* END TrajectoryProfileSinoid ******************************************************************************************/

