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

#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_lin.h>
#include <cob_cartesian_controller/Profile.h>


bool TrajectoryProfileGeneratorLin::calculateProfile(std::vector<double>* path_matrix,
                                                     const double Se, const double Se_roll, const double Se_pitch, const double Se_yaw,
                                                     cob_cartesian_controller::MoveLinStruct& move_lin)
{
    if(move_lin.profile.profile_type == cob_cartesian_controller::Profile::RAMP)
    {
        ROS_INFO("Ramp Profile");
        return calculateRampProfile(path_matrix, Se, Se_roll, Se_pitch, Se_yaw, move_lin);
    }
    else if(move_lin.profile.profile_type == cob_cartesian_controller::Profile::SINOID)
    {
        ROS_INFO("Sinoid Profile");
        return calculateSinoidProfile(path_matrix, Se, Se_roll, Se_pitch, Se_yaw, move_lin);
    }
    else
    {
        ROS_ERROR("Unknown Profile");
        return false;
    }
}

bool TrajectoryProfileGeneratorLin::calculateRampProfile(std::vector<double>* path_matrix,
                                                         const double Se, const double Se_roll, const double Se_pitch, const double Se_yaw,
                                                         cob_cartesian_controller::MoveLinStruct& move_lin)
{
    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;
    int steps_te, steps_tv, steps_tb = 0;
    double tv, tb, te = 0.0;
    double Se_max = 0.0;

    // If rotateOnly == true, then set the largest angular difference as Se_max.
    if(move_lin.rotate_only)
    {
        Se_max = std::max(std::fabs(Se), Se_max);
        Se_max = std::max(std::fabs(Se_roll), Se_max);
        Se_max = std::max(std::fabs(Se_pitch), Se_max);
        Se_max = std::max(std::fabs(Se_yaw), Se_max);
    }
    else    // Otherwise set the linear-path as Se_max
    {
        Se_max = Se;
    }

    // Calculate the Ramp Profile Parameters
    if (move_lin.profile.vel > sqrt(std::fabs(Se_max) * move_lin.profile.accl))
    {
        move_lin.profile.vel = sqrt(std::fabs(Se_max) * move_lin.profile.accl);
    }
    tb = move_lin.profile.vel / move_lin.profile.accl;
    te = (std::fabs(Se_max) / move_lin.profile.vel) + tb;
    tv = te - tb;

//ToDo: steps_tb is int! Do we need ceil() or floor()?
    // Interpolationsteps for every timesequence
    steps_tb = (double)tb      / t_ipo_;
    steps_tv = (double)(tv-tb) / t_ipo_;
    steps_te = (double)(te-tv) / t_ipo_;

//ToDo: What's the purpose of this? division by t_ipo_ followed by multiplication with t_ipo_?
    // Reconfigure timings wtih t_ipo_
    tb = steps_tb * t_ipo_;
    tv = (steps_tb + steps_tv) * t_ipo_;
    te = (steps_tb + steps_tv + steps_te) * t_ipo_;

    //Convert to RPY
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(move_lin.start.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Calculate the paths
    if(!generateRampPath(linear_path, move_lin.profile.vel, move_lin.profile.accl, Se_max, (steps_tb + steps_tv + steps_te)))
    {
        ROS_WARN("Error while Calculating linear_path");
        return false;
    }
    if(!generateRampPathWithTe(roll_path, te, move_lin.profile.accl, Se_roll, (steps_tb + steps_tv + steps_te), roll))
    {
        ROS_WARN("Error while Calculating roll_path");
        return false;
    }
    if(!generateRampPathWithTe(pitch_path, te, move_lin.profile.accl, Se_pitch, (steps_tb + steps_tv + steps_te), pitch))
    {
        ROS_WARN("Error while Calculating pitch_path");
        return false;
    }
    if(!generateRampPathWithTe(yaw_path, te, move_lin.profile.accl, Se_yaw, (steps_tb + steps_tv + steps_te), yaw))
    {
        ROS_WARN("Error while Calculating yaw_path");
        return false;
    }

    // Resize the path vectors
    unsigned int max_steps = 0;
    max_steps = std::max((unsigned int)linear_path.size(), max_steps);
    max_steps = std::max((unsigned int)roll_path.size(), max_steps);
    max_steps = std::max((unsigned int)pitch_path.size(), max_steps);
    max_steps = std::max((unsigned int)yaw_path.size(), max_steps);

    linear_path.resize(max_steps, linear_path.back());
    roll_path.resize(max_steps, roll_path.back());
    pitch_path.resize(max_steps, pitch_path.back());
    yaw_path.resize(max_steps, yaw_path.back());

    // Put the interpolated paths into the path_matrix
    path_matrix[LIN_INDEX] = linear_path;
    path_matrix[ROLL_INDEX] = roll_path;
    path_matrix[PITCH_INDEX] = pitch_path;
    path_matrix[YAW_INDEX] = yaw_path;

    return true;
}

bool TrajectoryProfileGeneratorLin::calculateSinoidProfile(std::vector<double>* path_matrix,
                                                           const double Se, const double Se_roll, const double Se_pitch, const double Se_yaw,
                                                           cob_cartesian_controller::MoveLinStruct& move_lin)
{
    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;
    int steps_te, steps_tv, steps_tb = 0;
    double tv, tb, te = 0.0;
    double Se_max = 0.0;

    // If rotateOnly == true, then set the largest angular difference as Se_max.
    if(move_lin.rotate_only)
    {
        Se_max = std::max(std::fabs(Se), Se_max);
        Se_max = std::max(std::fabs(Se_roll), Se_max);
        Se_max = std::max(std::fabs(Se_pitch), Se_max);
        Se_max = std::max(std::fabs(Se_yaw), Se_max);
    }
    else    // Otherwise set the linear-path as Se_max
    {
        Se_max = Se;
    }

    // Calculate the Sinoid Profile Parameters
    if (move_lin.profile.vel > sqrt(std::fabs(Se_max) * move_lin.profile.accl / 2))
    {
        move_lin.profile.vel = sqrt(std::fabs(Se_max) * move_lin.profile.accl / 2);
    }
    tb = 2 * move_lin.profile.vel / move_lin.profile.accl;
    te = (std::fabs(Se_max) / move_lin.profile.vel) + tb;
    tv = te - tb;

//ToDo: steps_tb is int! Do we need ceil() or floor()?
    // Interpolationsteps for every timesequence
    steps_tb = (double)tb      / t_ipo_;
    steps_tv = (double)(tv-tb) / t_ipo_;
    steps_te = (double)(te-tv) / t_ipo_;

//ToDo: What's the purpose of this? division by t_ipo_ followed by multiplication with t_ipo_?
    // Reconfigure timings wtih t_ipo_
    tb = steps_tb * t_ipo_;
    tv = (steps_tb + steps_tv) * t_ipo_;
    te = (steps_tb + steps_tv + steps_te) * t_ipo_;

    //Convert to RPY
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(move_lin.start.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Calculate the paths
    if(!generateSinoidPath(linear_path, move_lin.profile.vel, move_lin.profile.accl, Se_max, (steps_tb + steps_tv + steps_te)))
    {
        ROS_WARN("Error while Calculating linear_path");
        return false;
    }
    if(!generateSinoidPathWithTe(roll_path, te, move_lin.profile.accl, Se_roll, (steps_tb + steps_tv + steps_te), roll))
    {
        ROS_WARN("Error while Calculating roll_path");
        return false;
    }
    if(!generateSinoidPathWithTe(pitch_path, te, move_lin.profile.accl, Se_pitch, (steps_tb + steps_tv + steps_te), pitch))
    {
        ROS_WARN("Error while Calculating pitch_path");
        return false;
    }
    if(!generateSinoidPathWithTe(yaw_path, te, move_lin.profile.accl, Se_yaw, (steps_tb + steps_tv + steps_te), yaw))
    {
        ROS_WARN("Error while Calculating yaw_path");
        return false;
    }

    // Resize the path vectors
    unsigned int max_steps = 0;
    max_steps = std::max((unsigned int)linear_path.size(), max_steps);
    max_steps = std::max((unsigned int)roll_path.size(), max_steps);
    max_steps = std::max((unsigned int)pitch_path.size(), max_steps);
    max_steps = std::max((unsigned int)yaw_path.size(), max_steps);

    linear_path.resize(max_steps, linear_path.back());
    roll_path.resize(max_steps, roll_path.back());
    pitch_path.resize(max_steps, pitch_path.back());
    yaw_path.resize(max_steps, yaw_path.back());

    // Put the interpolated paths into the path_matrix
    path_matrix[LIN_INDEX] = linear_path;
    path_matrix[ROLL_INDEX] = roll_path;
    path_matrix[PITCH_INDEX] = pitch_path;
    path_matrix[YAW_INDEX] = yaw_path;

    return true;
}

bool TrajectoryProfileGeneratorLin::generateRampPath(std::vector<double>& path_array, double vel_max, double accl_max, const double Se_max, const int steps_max)
{
    double tv, tb, te = 0.0;
    int steps_te, steps_tv, steps_tb = 0;

    // Reconfigure the timings and parameters with t_ipo_
    tb = (vel_max/(accl_max * t_ipo_)) * t_ipo_;
    tv = (std::fabs(Se_max)/(vel_max * t_ipo_)) * t_ipo_;
    te = tv + tb;
    vel_max = std::fabs(Se_max) / tv;
    accl_max = vel_max / tb;

    // Calculate the Profile Timings for the longest path
    tb = vel_max/accl_max;
    te = (std::fabs(Se_max) / vel_max) + tb;
    tv = te - tb;

    // Interpolationsteps for every timesequence
    steps_tb = (double)tb      / t_ipo_;
    steps_tv = (double)(tv-tb) / t_ipo_;
    steps_te = (double)(te-tv) / t_ipo_;

    // Reconfigure timings wtih t_ipo_
    tb = steps_tb * t_ipo_;
    tv = (steps_tb + steps_tv) * t_ipo_;
    te = (steps_tb + steps_tv + steps_te) * t_ipo_;

    ROS_DEBUG("Vm: %f m/s", vel_max);
    ROS_DEBUG("Bm: %f m/s^-2", accl_max);
    ROS_DEBUG("Se_max: %f ", Se_max);
    ROS_DEBUG("tb: %f s", tb);
    ROS_DEBUG("tv: %f s", tv);
    ROS_DEBUG("te: %f s", te);

    // Calculate the ramp profile path
    // 0 <= t <= tb
    for(int i = 0 ; i <= steps_tb-1 ; i++)
    {
        path_array.push_back( Se_max/std::fabs(Se_max)*(0.5*accl_max*pow((t_ipo_*i),2)));
    }

    // tb <= t <= tv
    for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
    {
        path_array.push_back(Se_max/std::fabs(Se_max)*(vel_max*(t_ipo_*i)-0.5*pow(vel_max,2)/accl_max));
    }

    // tv <= t <= te
    for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
    {
        path_array.push_back(Se_max/std::fabs(Se_max)*(vel_max * (te-tb) - 0.5*accl_max* pow(te-(i*t_ipo_),2)));
    }

    return true;
}

bool TrajectoryProfileGeneratorLin::generateSinoidPath(std::vector<double>& path_array, double vel_max, double accl_max, const double Se_max, const int steps_max)
{
    double tv, tb, te = 0.0;
    int steps_te, steps_tv, steps_tb = 0;

    // Reconfigure the timings and parameters with t_ipo_
    tb = (vel_max/(accl_max * t_ipo_)) * t_ipo_;
    tv = (std::fabs(Se_max)/(vel_max * t_ipo_)) * t_ipo_;
    te = tv + tb;
    vel_max = std::fabs(Se_max) / tv;
    accl_max = vel_max / tb;

    // Calculate the Profile Timings for the longest path
    tb = 2 * vel_max/accl_max;
    te = (std::fabs(Se_max) / vel_max) + tb;
    tv = te - tb;

    // Interpolationsteps for every timesequence
    steps_tb = (double)tb      / t_ipo_;
    steps_tv = (double)(tv-tb) / t_ipo_;
    steps_te = (double)(te-tv) / t_ipo_;

    // Reconfigure timings wtih t_ipo_
    tb = steps_tb * t_ipo_;
    tv = (steps_tb + steps_tv) * t_ipo_;
    te = (steps_tb + steps_tv + steps_te) * t_ipo_;

    ROS_DEBUG("Vm: %f m/s", vel_max);
    ROS_DEBUG("Bm: %f m/s^-2", accl_max);
    ROS_DEBUG("Se_max: %f ", Se_max);
    ROS_DEBUG("tb: %f s", tb);
    ROS_DEBUG("tv: %f s", tv);
    ROS_DEBUG("te: %f s", te);

    // Calculate the sinoid profile path
    // 0 <= t <= tb
    for(int i = 0 ; i <= steps_tb-1 ; i++)
    {
        path_array.push_back( Se_max/std::fabs(Se_max)*( accl_max*(0.25*pow(i*t_ipo_,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*t_ipo_))-1))));
    }
    // tb <= t <= tv
    for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
    {
        path_array.push_back(Se_max/std::fabs(Se_max)*( vel_max*(i*t_ipo_-0.5*tb)));
    }
    // tv <= t <= te
    for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
    {
        path_array.push_back(Se_max/std::fabs(Se_max)*( 0.5 * accl_max *( te*(i*t_ipo_ + tb) - 0.5*(pow(i*t_ipo_,2)+pow(te,2)+2*pow(tb,2)) + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos( ((2*M_PI)/tb) * (i*t_ipo_-tv))))));
    }

    return true;
}

bool TrajectoryProfileGeneratorLin::generateRampPathWithTe(std::vector<double>& path_array, double te,
                                                           double accl_max, const double Se_max, const int steps_max, const double start_angle)
{
    double tv,tb = 0.0;
    int steps_te, steps_tv, steps_tb = 0;
    double vel_max;

    if(std::fabs(Se_max) > 0.001)
    {
        // Calculate the Profile Timings
        // Reconfigure accl_max and Velmax
        while(te < 2 * sqrt(std::fabs(Se_max)/accl_max))
        {
            accl_max+=0.001;
        }

        vel_max = accl_max * te / 2 - sqrt((pow(accl_max,2)*pow(te,2)/4) - std::fabs(Se_max) * accl_max );

        // Calculate profile timings, te is known
        tb = vel_max/accl_max;
        tv = te - tb;

        // Interpolationsteps for every timesequence
        steps_tb = (double)tb / t_ipo_;
        steps_tv = (double)(tv-tb) / t_ipo_;
        steps_te = (double)(te-tv) / t_ipo_;

        // Reconfigure timings wtih t_ipo_
        tb = steps_tb * t_ipo_;
        tv = (steps_tb + steps_tv) * t_ipo_;
        te = (steps_tb + steps_tv + steps_te) * t_ipo_;

        // Calculate the ramp profile path
        // 0 <= t <= tb
        for(int i = 0 ; i <= steps_tb-1 ; i++)
        {
            path_array.push_back( start_angle + Se_max/std::fabs(Se_max)*(0.5*accl_max*pow((t_ipo_*i),2)));
        }
        // tb <= t <= tv
        for(int i=steps_tb;i<=(steps_tb+steps_tv-1);i++)
        {
            path_array.push_back(start_angle + Se_max/std::fabs(Se_max)*(vel_max*(t_ipo_*i)-0.5*pow(vel_max,2)/accl_max));
        }
        // tv <= t <= te
        for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1);i++)
        {
            path_array.push_back(start_angle + Se_max/std::fabs(Se_max)*(vel_max * (te-tb) - 0.5*accl_max* pow(te-(i*t_ipo_),2)));
        }
    }
    else
    {
        path_array.push_back(start_angle);
    }
    return true;
}

bool TrajectoryProfileGeneratorLin::generateSinoidPathWithTe(std::vector<double>& path_array, double te,
                                                    double accl_max, const double Se_max, const int steps_max, const double start_angle)
{
    double tv,tb = 0.0;
    int steps_te, steps_tv, steps_tb = 0;
    double vel_max;

    if(std::fabs(Se_max) > 0.001)
    {
        // Calculate the Profile Timings
        // Reconfigure accl_max and Velmax
        while(te < sqrt(std::fabs(Se_max) * 8/accl_max))
        {
            accl_max += 0.001;
        }
        vel_max = accl_max * te / 4 - sqrt((pow(accl_max,2)*pow(te,2)/16) - std::fabs(Se_max) * accl_max/2 );

        // Calculate profile timings, te is known
        tb = 2 * vel_max/accl_max;
        tv = te - tb;

        // Interpolationsteps for every timesequence
        steps_tb = (double)tb / t_ipo_;
        steps_tv = (double)(tv-tb) / t_ipo_;
        steps_te = (double)(te-tv) / t_ipo_;

        // Reconfigure timings wtih t_ipo_
        tb = steps_tb * t_ipo_;
        tv = (steps_tb + steps_tv) * t_ipo_;
        te = (steps_tb + steps_tv + steps_te) * t_ipo_;

        // Calculate the sinoid profile path
        // 0 <= t <= tb
        for(int i = 0 ; i <= steps_tb-1 ; i++)
        {
            path_array.push_back(start_angle + Se_max/std::fabs(Se_max)*( accl_max*(0.25*pow(i*t_ipo_,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*t_ipo_))-1))));
        }
        // tb <= t <= tv
        for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
        {
            path_array.push_back(start_angle + Se_max/std::fabs(Se_max)*(vel_max*(i*t_ipo_-0.5*tb)));
        }
        // tv <= t <= te
        for(int i = (steps_tb + steps_tv); i < (steps_tv+steps_tb+steps_te-1) ; i++){
            path_array.push_back(start_angle + Se_max/std::fabs(Se_max)*(0.5*accl_max*( te*(i*t_ipo_ + tb) - 0.5*(pow(i*t_ipo_,2)+pow(te,2)+2*pow(tb,2))  + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos( ((2*M_PI)/tb) * (i*t_ipo_-tv))))));
        }
    }
    else
    {
        path_array.push_back(start_angle);
    }
    return true;
}
