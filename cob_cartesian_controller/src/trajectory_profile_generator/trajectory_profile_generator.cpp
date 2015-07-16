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

#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator.h>
#include <cob_cartesian_controller/Profile.h>

bool TrajectoryProfileGenerator::calculateProfile(std::vector<double>& path_array, double Se,
                                                  double vel_max, double accl_max, unsigned int profile)
{
    int steps_te, steps_tv, steps_tb = 0;
    double tv, tb, te = 0.0;
    double T_IPO = pow(update_rate_, -1);

    if(profile == cob_cartesian_controller::Profile::RAMP)
    {
        // Calculate the Ramp Profile Parameters
        if (vel_max > sqrt(Se*accl_max))
        {
            vel_max = sqrt(Se*accl_max);
        }
        tb = vel_max/accl_max;
        te = (Se / vel_max) + tb;
        tv = te - tb;
    }
    else if(profile == cob_cartesian_controller::Profile::SINOID)
    {
        // Calculate the Sinoide Profile Parameters
        if (vel_max > sqrt(Se*accl_max/2))
        {
            vel_max = sqrt(Se*accl_max/2);
        }
        tb = 2 * vel_max/accl_max;
        te = (Se / vel_max) + tb;
        tv = te - tb;
    }
    else
    {
        ROS_ERROR("Unknown Profile");
        return false;
    }

    // Interpolationsteps for every timesequence
    steps_tb = (double)tb / T_IPO;
    steps_tv = (double)(tv-tb) / T_IPO;
    steps_te = (double)(te-tv) / T_IPO;

    // Reconfigure timings wtih T_IPO
    tb = steps_tb * T_IPO;
    tv = (steps_tb + steps_tv) * T_IPO;
    te = (steps_tb + steps_tv + steps_te) * T_IPO;

//    ROS_INFO("Vm: %f m/s", vel_max);
//    ROS_INFO("Bm: %f m/s^-2", accl_max);
//    ROS_INFO("Se: %f ", Se);
//    ROS_INFO("tb: %f s", tb);
//    ROS_INFO("tv: %f s", tv);
//    ROS_INFO("te: %f s", te);

    if(cob_cartesian_controller::Profile::RAMP)
    {
        ROS_INFO("Ramp Profile");
        // Calculate the ramp profile path
        // 0 <= t <= tb
        for(int i = 0 ; i <= steps_tb-1 ; i++)
        {
            path_array.push_back(0.5 * accl_max * pow((T_IPO * i),2));
        }
        // tb <= t <= tv
        for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
        {
            path_array.push_back(vel_max * (T_IPO * i) - 0.5 * pow(vel_max, 2)/accl_max);
        }
        // tv <= t <= te
        for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
        {
            path_array.push_back(vel_max * (te-tb) - 0.5 * accl_max * pow(te - (T_IPO * i), 2));
        }
    }
    else if(cob_cartesian_controller::Profile::SINOID)
    {
        ROS_INFO("Sinoide Profile");
        // Calculate the sinoide profile path
        // 0 <= t <= tb
        for(int i = 0 ; i <= steps_tb-1 ; i++)
        {
            path_array.push_back(accl_max * (0.25 * pow(T_IPO * i, 2) + pow(tb, 2)/(8 * pow(M_PI, 2)) * (cos(2 * M_PI/tb * (T_IPO*i))-1)));
        }
        // tb <= t <= tv
        for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
        {
            path_array.push_back(vel_max * (i*T_IPO - 0.5 * tb));
        }
        // tv <= t <= te
        for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
        {
            path_array.push_back(0.5 * accl_max * (te * (i*T_IPO + tb) - 0.5 * (pow(T_IPO*i, 2) + pow(te, 2) + 2 * pow(tb, 2)) + (pow(tb, 2)/(4 * pow(M_PI, 2))) * (1 - cos( ((2*M_PI)/tb) * (i*T_IPO - tv)))));
        }
    }
    else
    {
        ROS_ERROR("Unknown Profile");
        return false;
    }
    return true;
}


bool TrajectoryProfileGenerator::calculateProfileForAngularMovements(std::vector<double>* path_matrix,
                                                                      double Se, double Se_roll, double Se_pitch, double Se_yaw,
                                                                      cob_cartesian_controller::MoveLinStruct& move_lin)
{
    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;
    int steps_te, steps_tv, steps_tb = 0;
    double tv, tb, te = 0.0;
    double T_IPO = pow(update_rate_, -1);
    double params[4][2];
    double Se_max, temp = 0.0;
    bool linear_okay, roll_okay, pitch_okay, yaw_okay = false;

    double Se_array[4] = {Se, Se_roll, Se_pitch, Se_yaw};

    for(int i = 0 ; i < sizeof(Se_array) ; i++)
    {
        if(temp < std::fabs(Se_array[i]))
        {
            temp = std::fabs(Se_array[i]);
        }
    }

    // If rotateOnly == true, then set the largest angular difference as Se_max.
    if(move_lin.rotate_only)
    {
        Se_max = temp;
    }
    else    // Otherwise set the linear-path as Se_max
    {
        Se_max = Se;
    }

    // Calculate the Profile Timings for the linear-path
    if(move_lin.profile.profile_type == cob_cartesian_controller::Profile::RAMP)
    {
        // Calculate the Ramp Profile Parameters
        if (move_lin.profile.vel > sqrt(std::fabs(Se_max) * move_lin.profile.accl))
        {
            move_lin.profile.vel = sqrt(std::fabs(Se_max) * move_lin.profile.accl);
        }
        tb = move_lin.profile.vel / move_lin.profile.accl;
        te = (std::fabs(Se_max) / move_lin.profile.vel) + tb;
        tv = te - tb;
    }
    else if(move_lin.profile.profile_type == cob_cartesian_controller::Profile::SINOID)
    {
        // Calculate the Sinoide Profile Parameters
        if (move_lin.profile.vel > sqrt(std::fabs(Se_max) * move_lin.profile.accl / 2))
        {
            move_lin.profile.vel = sqrt(std::fabs(Se_max) * move_lin.profile.accl / 2);
        }
        tb = 2 * move_lin.profile.vel / move_lin.profile.accl;
        te = (std::fabs(Se_max) / move_lin.profile.vel) + tb;
        tv = te - tb;
    }
    else
    {
        ROS_ERROR("Unknown Profile");
        return false;
    }

    // Interpolationsteps for every timesequence
    steps_tb = (double)tb      / T_IPO;
    steps_tv = (double)(tv-tb) / T_IPO;
    steps_te = (double)(te-tv) / T_IPO;

    // Reconfigure timings wtih T_IPO
    tb = steps_tb * T_IPO;
    tv = (steps_tb + steps_tv) * T_IPO;
    te = (steps_tb + steps_tv + steps_te) * T_IPO;

    // Calculate the paths
    if(!generatePath(linear_path, T_IPO, move_lin.profile.vel, move_lin.profile.accl, Se_max, (steps_tb + steps_tv + steps_te), move_lin.profile.profile_type))
    {
        ROS_WARN("Error while Calculating path");
        return false;
    }
    if(!generatePathWithTe(roll_path, T_IPO, te, move_lin.profile.accl, Se_roll, (steps_tb + steps_tv + steps_te), move_lin.roll, move_lin.profile.profile_type))
    {
        ROS_WARN("Error while Calculating path");
        return false;
    }
    if(!generatePathWithTe(pitch_path, T_IPO, te, move_lin.profile.accl, Se_pitch, (steps_tb + steps_tv + steps_te), move_lin.pitch, move_lin.profile.profile_type))
    {
        ROS_WARN("Error while Calculating path");
        return false;
    }
    if(!generatePathWithTe(yaw_path, T_IPO, te, move_lin.profile.accl, Se_yaw, (steps_tb + steps_tv + steps_te), move_lin.yaw, move_lin.profile.profile_type))
    {
        ROS_WARN("Error while Calculating path");
        return false;
    }

    // Get the Vector sizes of each path-vector
    int max_step_array[4], max_steps;

    max_step_array[0] = linear_path.size();
    max_step_array[1] = roll_path.size();
    max_step_array[2] = pitch_path.size();
    max_step_array[3] = yaw_path.size();

    // Get the largest one
    max_steps = 0;
    for(int i = 0 ; i < 4 ; i++)
    {
        if(max_steps<max_step_array[i])
        {
            max_steps = max_step_array[i];
        }
    }

    // Check if every vector has the same length than the largest one.
    while(true)
    {
        if(linear_path.size() < max_steps)
        {
            linear_path.push_back(linear_path.at(linear_path.size()-1));
        }
        else
        {
            linear_okay = true;
        }

        if(roll_path.size() < max_steps)
        {
            roll_path.push_back(roll_path.at(roll_path.size()-1));
        }
        else
        {
            roll_okay=true;
        }

        if(pitch_path.size() < max_steps)
        {
            pitch_path.push_back(pitch_path.at(pitch_path.size()-1));
        }
        else
        {
            pitch_okay=true;
        }

        if(yaw_path.size() < max_steps)
        {
            yaw_path.push_back(yaw_path.at(yaw_path.size()-1));
        }
        else
        {
            yaw_okay=true;
        }

        if(linear_okay && roll_okay && pitch_okay && yaw_okay)
        {
            break;
        }
    }

    // Put the interpolated paths into the path_matrix
    path_matrix[0] = linear_path;
    path_matrix[1] = roll_path;
    path_matrix[2] = pitch_path;
    path_matrix[3] = yaw_path;
    
    return true;
}


bool TrajectoryProfileGenerator::generatePath(std::vector<double>& path_array, double T_IPO,
                                              double vel_max, double accl_max, double Se_max, int steps_max, unsigned int profile)
{
    double tv, tb, te = 0.0;
    int steps_te, steps_tv, steps_tb = 0;

    // Reconfigure the timings and parameters with T_IPO
    tb = (vel_max/(accl_max * T_IPO)) * T_IPO;
    tv = (std::fabs(Se_max)/(vel_max * T_IPO)) * T_IPO;
    te = tv + tb;
    vel_max = std::fabs(Se_max) / tv;
    accl_max = vel_max / tb;

    // Calculate the Profile Timings for the longest path
    if(profile == cob_cartesian_controller::Profile::RAMP)
    {
        tb = vel_max/accl_max;
        te = (std::fabs(Se_max) / vel_max) + tb;
        tv = te - tb;
    }
    else if(profile == cob_cartesian_controller::Profile::SINOID)
    {
        tb = 2 * vel_max/accl_max;
        te = (std::fabs(Se_max) / vel_max) + tb;
        tv = te - tb;
    }
    else
    {
        ROS_ERROR("Unknown Profile");
        return false;
    }

    // Interpolationsteps for every timesequence
    steps_tb = (double)tb      / T_IPO;
    steps_tv = (double)(tv-tb) / T_IPO;
    steps_te = (double)(te-tv) / T_IPO;

    // Reconfigure timings wtih T_IPO
    tb = steps_tb * T_IPO;
    tv = (steps_tb + steps_tv) * T_IPO;
    te = (steps_tb + steps_tv + steps_te) * T_IPO;

    if(profile == cob_cartesian_controller::Profile::RAMP)
    {
        // Calculate the ramp profile path
        // 0 <= t <= tb
        for(int i = 0 ; i <= steps_tb-1 ; i++)
        {
            path_array.push_back( Se_max/std::fabs(Se_max)*(0.5*accl_max*pow((T_IPO*i),2)));
        }

        // tb <= t <= tv
        for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
        {
            path_array.push_back(Se_max/std::fabs(Se_max)*(vel_max*(T_IPO*i)-0.5*pow(vel_max,2)/accl_max));
        }

        // tv <= t <= te
        for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
        {
            path_array.push_back(Se_max/std::fabs(Se_max)*(vel_max * (te-tb) - 0.5*accl_max* pow(te-(i*T_IPO),2)));
        }
    }
    else if(profile == cob_cartesian_controller::Profile::SINOID)
    {
        // Calculate the sinoide profile path
        // 0 <= t <= tb
        for(int i = 0 ; i <= steps_tb-1 ; i++)
        {
            path_array.push_back( Se_max/std::fabs(Se_max)*( accl_max*(0.25*pow(i*T_IPO,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*T_IPO))-1))));
        }
        // tb <= t <= tv
        for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
        {
            path_array.push_back(Se_max/std::fabs(Se_max)*( vel_max*(i*T_IPO-0.5*tb)));
        }
        // tv <= t <= te
        for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
        {
            path_array.push_back(Se_max/std::fabs(Se_max)*( 0.5 * accl_max *( te*(i*T_IPO + tb) - 0.5*(pow(i*T_IPO,2)+pow(te,2)+2*pow(tb,2)) + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos( ((2*M_PI)/tb) * (i*T_IPO-tv))))));
        }
    }
    else
    {
        ROS_ERROR("Unknown Profile");
        return false;
    }
    return true;
}

bool TrajectoryProfileGenerator::generatePathWithTe(std::vector<double>& path_array, double T_IPO, double te, 
                                                    double accl_max, double Se_max, int steps_max, double start_angle, unsigned int profile)
{
    double tv,tb = 0.0;
    int steps_te, steps_tv, steps_tb = 0;
    double vel_max;

    if(std::fabs(Se_max) > 0.001)
    {
        // Calculate the Profile Timings
        if(profile == cob_cartesian_controller::Profile::RAMP)
        {
            // Reconfigure accl_max and Velmax
            while(te < 2 * sqrt(std::fabs(Se_max)/accl_max))
            {
                accl_max+=0.001;
            }

            vel_max = accl_max * te / 2 - sqrt((pow(accl_max,2)*pow(te,2)/4) - std::fabs(Se_max) * accl_max );

            // Calculate profile timings, te is known
            tb = vel_max/accl_max;
            tv = te - tb;
        }
        else if(profile == cob_cartesian_controller::Profile::SINOID)
        {
            // Reconfigure accl_max and Velmax
            while(te < sqrt(std::fabs(Se_max) * 8/accl_max))
            {
                accl_max += 0.001;
            }
            vel_max = accl_max * te / 4 - sqrt((pow(accl_max,2)*pow(te,2)/16) - std::fabs(Se_max) * accl_max/2 );

            // Calculate profile timings, te is known
            tb = 2 * vel_max/accl_max;
            tv = te - tb;
        }
        else
        {
            ROS_ERROR("Unknown Profile");
            return false;
        }

        // Interpolationsteps for every timesequence
        steps_tb = (double)tb / T_IPO;
        steps_tv = (double)(tv-tb) / T_IPO;
        steps_te = (double)(te-tv) / T_IPO;

        // Reconfigure timings wtih T_IPO
        tb = steps_tb * T_IPO;
        tv = (steps_tb + steps_tv) * T_IPO;
        te = (steps_tb + steps_tv + steps_te) * T_IPO;

        if(profile == cob_cartesian_controller::Profile::RAMP)
        {
            // Calculate the ramp profile path
            // 0 <= t <= tb
            for(int i = 0 ; i <= steps_tb-1 ; i++)
            {
                path_array.push_back( start_angle + Se_max/std::fabs(Se_max)*(0.5*accl_max*pow((T_IPO*i),2)));
            }
            // tb <= t <= tv
            for(int i=steps_tb;i<=(steps_tb+steps_tv-1);i++)
            {
                path_array.push_back(start_angle + Se_max/std::fabs(Se_max)*(vel_max*(T_IPO*i)-0.5*pow(vel_max,2)/accl_max));
            }
            // tv <= t <= te
            for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1);i++)
            {
                path_array.push_back(start_angle + Se_max/std::fabs(Se_max)*(vel_max * (te-tb) - 0.5*accl_max* pow(te-(i*T_IPO),2)));
            }
        }
        else if(profile == cob_cartesian_controller::Profile::SINOID)
        {
            // Calculate the sinoide profile path
            // 0 <= t <= tb
            for(int i = 0 ; i <= steps_tb-1 ; i++)
            {
                path_array.push_back(start_angle + Se_max/std::fabs(Se_max)*( accl_max*(0.25*pow(i*T_IPO,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*T_IPO))-1))));
            }
            // tb <= t <= tv
            for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
            {
                path_array.push_back(start_angle + Se_max/std::fabs(Se_max)*(vel_max*(i*T_IPO-0.5*tb)));
            }
            // tv <= t <= te
            for(int i = (steps_tb + steps_tv); i < (steps_tv+steps_tb+steps_te-1) ; i++){
                path_array.push_back(start_angle + Se_max/std::fabs(Se_max)*(0.5*accl_max*( te*(i*T_IPO + tb) - 0.5*(pow(i*T_IPO,2)+pow(te,2)+2*pow(tb,2))  + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos( ((2*M_PI)/tb) * (i*T_IPO-tv))))));
            }
        }
        else
        {
            ROS_ERROR("Unknown Profile");
            return false;
        }
    }
    else
    {
        path_array.push_back(start_angle);
    }
    return true;
}
