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
#include "cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_ramp.h"
/* BEGIN TrajectoryProfileRamp ********************************************************************************************/
inline cob_cartesian_controller::ProfileTimings TrajectoryProfileRamp::getProfileTimings(double Se, double te, double accl, double vel)
{
    CartesianControllerUtils utils;
    cob_cartesian_controller::ProfileTimings pt;
    int steps_te, steps_tv, steps_tb = 0;
    double tv, tb = 0.0;

    // Calculate the Sinoid Profile Parameters
    if (vel > sqrt(std::fabs(Se) * accl))
    {
        vel = sqrt(std::fabs(Se) * accl);
    }

    //Todo: Find a value.
    if(vel > 0.001)
    {
        tb = utils.roundUpToMultiplier(vel / accl, params_.profile.t_ipo);
        if(te == 0)
        {
            te = utils.roundUpToMultiplier((std::fabs(Se) / vel) + tb,  params_.profile.t_ipo);
        }
        tv = te - tb;

        ROS_INFO_STREAM("=========================================================================");
        ROS_INFO_STREAM("vel_new:" << vel << " accl: " << accl << " Se: " << std::fabs(Se));
        ROS_INFO_STREAM("tb:" << tb << " tv: " << tv << " te: " << te);

        // Interpolationsteps for every timesequence
        pt.tb = tb;
        pt.tv = tv;
        pt.te = te;

        pt.steps_tb = pt.tb/params_.profile.t_ipo;
        pt.steps_tv = (pt.tv-pt.tb)/params_.profile.t_ipo;
        pt.steps_te = (pt.te-pt.tv)/params_.profile.t_ipo;

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


inline bool TrajectoryProfileRamp::generatePath(cob_cartesian_controller::PathArray &pa)
{
    std::vector<double> array;
    cob_cartesian_controller::ProfileTimings pt;
    double accl_max = params_.profile.accl;
    double vel_max = params_.profile.vel;

    // Calculate the Profile Timings
    pt = getProfileTimings(pa.Se_, pt_max_.te, accl_max, vel_max);
    if(pt.ok)
    {
        array = getTrajectory(pa.start_value_ , std::fabs(pa.Se_), accl_max, vel_max, params_.profile.t_ipo, pt.steps_tb, pt.steps_tv, pt.steps_te, pt.tb, pt.tv, pt.te);
    }
    else
    {
        array.push_back(pa.start_value_);
    }

    pa.array_ = array;
    return true;
}

inline std::vector<double> TrajectoryProfileRamp::getTrajectory(double start_value, double se, double accl, double vel, double t_ipo, double steps_tb, double steps_tv, double steps_te, double tb, double tv, double te)
{
    std::vector<double> array;
    unsigned int i = 1;
    double direction = se/std::fabs(se);

    // Calculate the ramp profile path
    // 0 <= t <= tb
    for(; i <= steps_tb ; i++)
    {
        array.push_back(start_value + direction * (0.5*accl*pow((t_ipo*i),2)));
    }
    // tb <= t <= tv
    for(; i <= (steps_tb + steps_tv); i++)
    {
        array.push_back(start_value + direction *(vel*(t_ipo*i) - 0.5*pow(vel,2)/accl));
    }
    // tv <= t <= te
    for(; i <= (steps_tv + steps_tb + steps_te + 1); i++)
    {
        array.push_back(start_value + direction * (vel * tv - 0.5 * accl * pow(te-(t_ipo*i),2)));
    }

    return array;
}

inline bool TrajectoryProfileRamp::calculateProfile(std::vector<double> path_matrix[4],
                                                    const double Se, const double Se_roll, const double Se_pitch, const double Se_yaw,
                                                    geometry_msgs::Pose start)
{
    CartesianControllerUtils ccu;
    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;
    double roll_start, pitch_start, yaw_start;

    //Convert to RPY
    tf::Quaternion q;
    tf::quaternionMsgToTF(start.orientation, q);
    tf::Matrix3x3(q).getRPY(roll_start, pitch_start, yaw_start);

    cob_cartesian_controller::PathArray lin(0, Se, 0, linear_path);
    cob_cartesian_controller::PathArray roll(1, Se_roll, roll_start, roll_path);
    cob_cartesian_controller::PathArray pitch(2, Se_pitch, pitch_start, pitch_path);
    cob_cartesian_controller::PathArray yaw(3, Se_yaw, yaw_start, yaw_path);

    cob_cartesian_controller::PathMatrix pm(lin,roll,pitch,yaw);

    // Get the profile timings from the longest path
    pt_max_ = getProfileTimings(pm.getMaxSe(), 0, params_.profile.accl, params_.profile.vel);

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

/* END TrajectoryProfileRamp **********************************************************************************************/
