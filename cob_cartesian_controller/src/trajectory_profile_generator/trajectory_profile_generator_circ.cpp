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

#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_circ.h>
#include <cob_cartesian_controller/Profile.h>


bool TrajectoryProfileGeneratorCirc::calculateProfile(std::vector<double>& path_array, const double Se, cob_cartesian_controller::ProfileStruct& profile)
{
    if(profile.profile_type == cob_cartesian_controller::Profile::RAMP)
    {
        ROS_INFO("Ramp Profile");
        return calculateRampProfile(path_array, Se, profile);
    }
    else if(profile.profile_type == cob_cartesian_controller::Profile::SINOID)
    {
        ROS_INFO("Sinoid Profile");
        return calculateSinoidProfile(path_array, Se, profile);
    }
    else
    {
        ROS_ERROR("Unknown Profile");
        return false;
    }
}

bool TrajectoryProfileGeneratorCirc::calculateRampProfile(std::vector<double>& path_array, const double Se, cob_cartesian_controller::ProfileStruct& profile)
{
    double vel_max = profile.vel;
    double accl_max = profile.accl;
    int steps_te, steps_tv, steps_tb = 0;
    double tv, tb, te = 0.0;

    // Calculate the Ramp Profile Parameters
    if (vel_max > sqrt(Se*accl_max))
    {
        vel_max = sqrt(Se*accl_max);
    }
    tb = vel_max/accl_max;
    te = (Se / vel_max) + tb;
    tv = te - tb;

//ToDo: steps_tb is int! Do we need ceil() or floor()?
    // Interpolationsteps for every timesequence
    steps_tb = (double)tb / t_ipo_;
    steps_tv = (double)(tv-tb) / t_ipo_;
    steps_te = (double)(te-tv) / t_ipo_;

//ToDo: What's the purpose of this? division by t_ipo_ followed by multiplication with t_ipo_?
    // Reconfigure timings wtih t_ipo_
    tb = steps_tb * t_ipo_;
    tv = (steps_tb + steps_tv) * t_ipo_;
    te = (steps_tb + steps_tv + steps_te) * t_ipo_;

    ROS_DEBUG("Vm: %f m/s", vel_max);
    ROS_DEBUG("Bm: %f m/s^-2", accl_max);
    ROS_DEBUG("Se: %f ", Se);
    ROS_DEBUG("tb: %f s", tb);
    ROS_DEBUG("tv: %f s", tv);
    ROS_DEBUG("te: %f s", te);

//ToDo: Verify for-conditions: 0 <= t < steps_tb.?
    // Calculate the ramp profile path
    // 0 <= t <= tb
    for(int i = 0 ; i <= steps_tb-1 ; i++)
    {
        path_array.push_back(0.5 * accl_max * pow((t_ipo_ * i),2));
    }
    // tb <= t <= tv
    for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
    {
        path_array.push_back(vel_max * (t_ipo_ * i) - 0.5 * pow(vel_max, 2)/accl_max);
    }
    // tv <= t <= te
    for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
    {
        path_array.push_back(vel_max * (te-tb) - 0.5 * accl_max * pow(te - (t_ipo_ * i), 2));
    }

    return true;
}

bool TrajectoryProfileGeneratorCirc::calculateSinoidProfile(std::vector<double>& path_array, const double Se, cob_cartesian_controller::ProfileStruct& profile)
{
    double vel_max = profile.vel;
    double accl_max = profile.accl;
    int steps_te, steps_tv, steps_tb = 0;
    double tv, tb, te = 0.0;

    // Calculate the Sinoid Profile Parameters
    if (vel_max > sqrt(Se*accl_max/2))
    {
        vel_max = sqrt(Se*accl_max/2);
    }
    tb = 2 * vel_max/accl_max;
    te = (Se / vel_max) + tb;
    tv = te - tb;

//ToDo: steps_tb is int! Do we need ceil() or floor()?
    // Interpolationsteps for every timesequence
    steps_tb = (double)tb / t_ipo_;
    steps_tv = (double)(tv-tb) / t_ipo_;
    steps_te = (double)(te-tv) / t_ipo_;

//ToDo: What's the purpose of this? division by t_ipo_ followed by multiplication with t_ipo_?
    // Reconfigure timings wtih t_ipo_
    tb = steps_tb * t_ipo_;
    tv = (steps_tb + steps_tv) * t_ipo_;
    te = (steps_tb + steps_tv + steps_te) * t_ipo_;

    ROS_DEBUG("Vm: %f m/s", vel_max);
    ROS_DEBUG("Bm: %f m/s^-2", accl_max);
    ROS_DEBUG("Se: %f ", Se);
    ROS_DEBUG("tb: %f s", tb);
    ROS_DEBUG("tv: %f s", tv);
    ROS_DEBUG("te: %f s", te);

//ToDo: Verify for-conditions: 0 <= t < steps_tb.?
    // Calculate the sinoid profile path
    // 0 <= t <= tb
    for(int i = 0 ; i <= steps_tb-1 ; i++)
    {
        path_array.push_back(accl_max * (0.25 * pow(t_ipo_ * i, 2) + pow(tb, 2)/(8 * pow(M_PI, 2)) * (cos(2 * M_PI/tb * (t_ipo_*i))-1)));
    }
    // tb <= t <= tv
    for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
    {
        path_array.push_back(vel_max * (i*t_ipo_ - 0.5 * tb));
    }
    // tv <= t <= te
    for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
    {
        path_array.push_back(0.5 * accl_max * (te * (i*t_ipo_ + tb) - 0.5 * (pow(t_ipo_*i, 2) + pow(te, 2) + 2 * pow(tb, 2)) + (pow(tb, 2)/(4 * pow(M_PI, 2))) * (1 - cos( ((2*M_PI)/tb) * (i*t_ipo_ - tv)))));
    }

    return true;
}
