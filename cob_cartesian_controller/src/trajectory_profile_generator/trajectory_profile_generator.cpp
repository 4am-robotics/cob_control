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

void TrajectoryProfileGenerator::calculateProfile(std::vector<double> &pathArray, double Se, double VelMax,
                                                  double AcclMax, std::string profile)
{
    int steps_te, steps_tv, steps_tb = 0;
    double tv, tb, te = 0;
    double T_IPO=pow(update_rate_,-1);

    if(profile == "ramp")
    {
        // Calculate the Ramp Profile Parameters
        if (VelMax > sqrt(Se*AcclMax)){
            VelMax = sqrt(Se*AcclMax);
        }
        tb = VelMax/AcclMax;
        te = (Se / VelMax) + tb;
        tv = te - tb;
    }
    else if(profile == "sinoide")
    {
        // Calculate the Sinoide Profile Parameters
        if (VelMax > sqrt(Se*AcclMax/2))
        {
            VelMax = sqrt(Se*AcclMax/2);
        }
        tb = 2 * VelMax/AcclMax;
        te = (Se / VelMax) + tb;
        tv = te - tb;
    }
    else
    {
        ROS_ERROR("Unknown Profile");
    }

    // Interpolationsteps for every timesequence
    steps_tb = (double)tb / T_IPO;
    steps_tv = (double)(tv-tb) / T_IPO;
    steps_te = (double)(te-tv) / T_IPO;

    // Reconfigure timings wtih T_IPO
    tb = steps_tb * T_IPO;
    tv = (steps_tb + steps_tv) * T_IPO;
    te = (steps_tb + steps_tv + steps_te) * T_IPO;

//    ROS_INFO("Vm: %f m/s",VelMax);
//    ROS_INFO("Bm: %f m/s²",AcclMax);
//    ROS_INFO("Se: %f ",Se);
//    ROS_INFO("tb: %f s",tb);
//    ROS_INFO("tv: %f s",tv);
//    ROS_INFO("te: %f s",te);

    if(profile == "ramp")
    {
        ROS_INFO("Ramp Profile");
        // Calculate the ramp profile path
        // 0 <= t <= tb
        for(int i = 0 ; i <= steps_tb-1 ; i++)
        {
            pathArray.push_back(0.5*AcclMax*pow((T_IPO*i),2));
        }
        // tb <= t <= tv
        for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
        {
            pathArray.push_back(VelMax*(T_IPO*i)-0.5*pow(VelMax,2)/AcclMax);
        }
        // tv <= t <= te
        for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
        {
            pathArray.push_back(VelMax * (te-tb) - 0.5*AcclMax* pow(te-(i*T_IPO),2));
        }
    }
    else if(profile == "sinoide")
    {
        ROS_INFO("Sinoide Profile");
        // Calculate the sinoide profile path
        // 0 <= t <= tb
        for(int i = 0 ; i <= steps_tb-1 ; i++)
        {
            pathArray.push_back(  AcclMax*(0.25*pow(i*T_IPO,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*T_IPO))-1)));
        }
        // tb <= t <= tv
        for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
        {
            pathArray.push_back(VelMax*(i*T_IPO-0.5*tb));
        }
        // tv <= t <= te
        for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
        {
            pathArray.push_back(0.5*AcclMax*( te*(i*T_IPO + tb) - 0.5*(pow(i*T_IPO,2)+pow(te,2)+2*pow(tb,2))  + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos( ((2*M_PI)/tb) * (i*T_IPO-tv)))));
        }
    }
    else
    {
        ROS_ERROR("Unknown Profile");
    }
}


void TrajectoryProfileGenerator::calculateProfileForAngularMovements(  std::vector<double> *pathMatrix,
                                                                       double Se, double Se_roll, double Se_pitch, double Se_yaw,
                                                                       double start_angle_roll, double start_angle_pitch, double start_angle_yaw,
                                                                       double VelMax, double AcclMax, std::string profile, bool justRotate)
{
    std::vector<double> linearPath, rollPath, pitchPath, yawPath;
    int steps_te, steps_tv, steps_tb = 0;
    double tv, tb, te = 0;
    double T_IPO = pow(update_rate_,-1);
    double params[4][2];
    double Se_max, temp = 0;
    bool linearOkay, rollOkay, pitchOkay, yawOkay = false;

    double Se_array[4] = {Se, Se_roll, Se_pitch, Se_yaw};

    for(int i = 0 ; i < sizeof(Se_array) ; i++)
    {
        if(temp < std::fabs(Se_array[i]))
        {
            temp = std::fabs(Se_array[i]);
        }
    }

    // If justRoate == true, then set the largest angular difference as Se_max.
    if(justRotate)
    {
        Se_max = temp;
    }
    else    // Otherwise set the linear-path as Se_max
    {
        Se_max = Se;
    }

    // Calculate the Profile Timings for the linear-path
    if(profile == "ramp")
    {
        // Calculate the Ramp Profile Parameters
        if (VelMax > sqrt(std::fabs(Se_max)*AcclMax))
        {
            VelMax = sqrt(std::fabs(Se_max)*AcclMax);
        }
        tb = VelMax/AcclMax;
        te = (std::fabs(Se_max) / VelMax) + tb;
        tv = te - tb;
    }
    else if(profile == "sinoide")
    {
        // Calculate the Sinoide Profile Parameters
        if (VelMax > sqrt(std::fabs(Se_max)*AcclMax/2))
        {
            VelMax = sqrt(std::fabs(Se_max)*AcclMax/2);
        }
        tb = 2 * VelMax/AcclMax;
        te = (std::fabs(Se_max) / VelMax) + tb;
        tv = te - tb;
    }
    else
    {
        ROS_ERROR("Unknown Profile");
    }

    // Interpolationsteps for every timesequence
    steps_tb = (double)tb / T_IPO;
    steps_tv = (double)(tv-tb) / T_IPO;
    steps_te = (double)(te-tv) / T_IPO;

    // Reconfigure timings wtih T_IPO
    tb = steps_tb * T_IPO;
    tv = (steps_tb + steps_tv) * T_IPO;
    te = (steps_tb + steps_tv + steps_te) * T_IPO;

    // Calculate the paths
    if(!generatePath(linearPath,       T_IPO,VelMax, AcclMax   , Se_max    , (steps_tb+steps_tv+steps_te), profile))
    {
        ROS_WARN("Error while Calculating path");
    }
    if(!generatePathWithTe(rollPath,   T_IPO, te   , AcclMax   , Se_roll   , (steps_tb+steps_tv+steps_te), start_angle_roll,   profile))
    {
        ROS_WARN("Error while Calculating path");
    }
    if(!generatePathWithTe(pitchPath,  T_IPO, te   , AcclMax   , Se_pitch  , (steps_tb+steps_tv+steps_te), start_angle_pitch,  profile))
    {
        ROS_WARN("Error while Calculating path");
    }
    if(!generatePathWithTe(yawPath,    T_IPO, te   , AcclMax   , Se_yaw    , (steps_tb+steps_tv+steps_te), start_angle_yaw,    profile))
    {
        ROS_WARN("Error while Calculating path");
    }

    // Get the Vector sizes of each path-vector
    int MaxStepArray[4], maxSteps;

    MaxStepArray[0] = linearPath.size();
    MaxStepArray[1] = rollPath.size();
    MaxStepArray[2] = pitchPath.size();
    MaxStepArray[3] = yawPath.size();

    // Get the largest one
    maxSteps = 0;
    for(int i = 0 ; i < 4 ; i++)
    {
        if(maxSteps<MaxStepArray[i])
            maxSteps=MaxStepArray[i];
    }

    // Check if every vector has the same length than the largest one.
    while(true)
    {
        if(linearPath.size() < maxSteps)
        {
            linearPath.push_back(linearPath.at(linearPath.size()-1));
        }
        else
        {
            linearOkay=true;
        }

        if(rollPath.size() < maxSteps)
        {
            rollPath.push_back(rollPath.at(rollPath.size()-1));
        }
        else
        {
            rollOkay=true;
        }

        if(pitchPath.size() < maxSteps)
        {
            pitchPath.push_back(pitchPath.at(pitchPath.size()-1));
        }
        else
        {
            pitchOkay=true;
        }

        if(yawPath.size() < maxSteps)
        {
            yawPath.push_back(yawPath.at(yawPath.size()-1));
        }
        else
        {
            yawOkay=true;
        }

        if(linearOkay && rollOkay && pitchOkay && yawOkay)
        {
            break;
        }
    }

    // Put the interpolated paths into the pathMatrix
    pathMatrix[0] = linearPath;
    pathMatrix[1] = rollPath;
    pathMatrix[2] = pitchPath;
    pathMatrix[3] = yawPath;
}


bool TrajectoryProfileGenerator::generatePath(std::vector<double> &pathArray, double T_IPO, double VelMax,
                                              double AcclMax, double Se_max, int steps_max, std::string profile)
{
    double tv,tb,te=0;
    int steps_te, steps_tv, steps_tb = 0;

    // Reconfigure the timings and parameters with T_IPO
    tb = (VelMax/(AcclMax*T_IPO)) * T_IPO;
    tv = (std::fabs(Se_max)/(VelMax * T_IPO)) * T_IPO;
    te = tv + tb;
    VelMax = std::fabs(Se_max) / tv;
    AcclMax = VelMax / tb;

    // Calculate the Profile Timings for the longest path
    if(profile == "ramp")
    {
        tb = VelMax/AcclMax;
        te = (std::fabs(Se_max) / VelMax) + tb;
        tv = te - tb;
    }
    else
    {
        tb = 2 * VelMax/AcclMax;
        te = (std::fabs(Se_max) / VelMax) + tb;
        tv = te - tb;
    }

    // Interpolationsteps for every timesequence
    steps_tb = (double)tb / T_IPO;
    steps_tv = (double)(tv-tb) / T_IPO;
    steps_te = (double)(te-tv) / T_IPO;

    // Reconfigure timings wtih T_IPO
    tb = steps_tb * T_IPO;
    tv = (steps_tb + steps_tv) * T_IPO;
    te = (steps_tb + steps_tv + steps_te) * T_IPO;

    if(profile == "ramp")
    {
        // Calculate the ramp profile path
        // 0 <= t <= tb
        for(int i = 0 ; i <= steps_tb-1 ; i++)
        {
            pathArray.push_back( Se_max/std::fabs(Se_max)*(0.5*AcclMax*pow((T_IPO*i),2)));
        }

        // tb <= t <= tv
        for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
        {
            pathArray.push_back(Se_max/std::fabs(Se_max)*(VelMax*(T_IPO*i)-0.5*pow(VelMax,2)/AcclMax));
        }

        // tv <= t <= te
        for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
        {
            pathArray.push_back(Se_max/std::fabs(Se_max)*(VelMax * (te-tb) - 0.5*AcclMax* pow(te-(i*T_IPO),2)));
        }
    }
    else if(profile == "sinoide")
    {
        // Calculate the sinoide profile path
        // 0 <= t <= tb
        for(int i = 0 ; i <= steps_tb-1 ; i++)
        {
            pathArray.push_back( Se_max/std::fabs(Se_max)*( AcclMax*(0.25*pow(i*T_IPO,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*T_IPO))-1))));
        }
        // tb <= t <= tv
        for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
        {
            pathArray.push_back(Se_max/std::fabs(Se_max)*( VelMax*(i*T_IPO-0.5*tb)));
        }
        // tv <= t <= te
        for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1) ; i++)
        {
            pathArray.push_back(Se_max/std::fabs(Se_max)*( 0.5 * AcclMax *( te*(i*T_IPO + tb) - 0.5*(pow(i*T_IPO,2)+pow(te,2)+2*pow(tb,2)) + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos( ((2*M_PI)/tb) * (i*T_IPO-tv))))));
        }
    }
    else
    {
        ROS_ERROR("Unknown Profile");
        return false;
    }
    return true;
}

bool TrajectoryProfileGenerator::generatePathWithTe(std::vector<double> &pathArray, double T_IPO, double te, double AcclMax,
                                                    double Se_max, int steps_max, double start_angle, std::string profile)
{
    double tv,tb=0;
    int steps_te,steps_tv,steps_tb=0;
    double VelMax;

    if(std::fabs(Se_max) > 0.001)
    {
        // Calculate the Profile Timings
        if(profile == "ramp")
        {
            // Reconfigure AcclMax and Velmax
            while(te < 2 * sqrt(std::fabs(Se_max)/AcclMax))
            {
                AcclMax+=0.001;
            }

            VelMax = AcclMax * te / 2 - sqrt((pow(AcclMax,2)*pow(te,2)/4) - std::fabs(Se_max) * AcclMax );

            // Calculate profile timings, te is known
            tb = VelMax/AcclMax;
            tv = te - tb;
        }
        else if(profile == "sinoide")
        {
            // Reconfigure AcclMax and Velmax
            while(te < sqrt(std::fabs(Se_max) * 8/AcclMax))
            {
                AcclMax += 0.001;
            }
            VelMax = AcclMax * te / 4 - sqrt((pow(AcclMax,2)*pow(te,2)/16) - std::fabs(Se_max) * AcclMax/2 );

            // Calculate profile timings, te is known
            tb = 2 * VelMax/AcclMax;
            tv = te - tb;
        }
        else
        {
            ROS_ERROR("Unknown Profile");
        }

        // Interpolationsteps for every timesequence
        steps_tb = (double)tb / T_IPO;
        steps_tv = (double)(tv-tb) / T_IPO;
        steps_te = (double)(te-tv) / T_IPO;

        // Reconfigure timings wtih T_IPO
        tb = steps_tb * T_IPO;
        tv = (steps_tb + steps_tv) * T_IPO;
        te = (steps_tb + steps_tv + steps_te) * T_IPO;

        if(profile == "ramp")
        {
            // Calculate the ramp profile path
            // 0 <= t <= tb
            for(int i = 0 ; i <= steps_tb-1 ; i++)
            {
                pathArray.push_back( start_angle + Se_max/std::fabs(Se_max)*(0.5*AcclMax*pow((T_IPO*i),2)));
            }
            // tb <= t <= tv
            for(int i=steps_tb;i<=(steps_tb+steps_tv-1);i++)
            {
                pathArray.push_back(start_angle + Se_max/std::fabs(Se_max)*(VelMax*(T_IPO*i)-0.5*pow(VelMax,2)/AcclMax));
            }
            // tv <= t <= te
            for(int i = (steps_tb + steps_tv) ; i < (steps_tv + steps_tb + steps_te-1);i++)
            {
                pathArray.push_back(start_angle + Se_max/std::fabs(Se_max)*(VelMax * (te-tb) - 0.5*AcclMax* pow(te-(i*T_IPO),2)));
            }
        }
        else if(profile == "sinoide")
        {
            // Calculate the sinoide profile path
            // 0 <= t <= tb
            for(int i = 0 ; i <= steps_tb-1 ; i++)
            {
                pathArray.push_back(start_angle + Se_max/std::fabs(Se_max)*( AcclMax*(0.25*pow(i*T_IPO,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*T_IPO))-1))));
            }
            // tb <= t <= tv
            for(int i = steps_tb ; i <= (steps_tb + steps_tv-1) ; i++)
            {
                pathArray.push_back(start_angle + Se_max/std::fabs(Se_max)*(VelMax*(i*T_IPO-0.5*tb)));
            }
            // tv <= t <= te
            for(int i = (steps_tb + steps_tv); i < (steps_tv+steps_tb+steps_te-1) ; i++){
                pathArray.push_back(start_angle + Se_max/std::fabs(Se_max)*(0.5*AcclMax*( te*(i*T_IPO + tb) - 0.5*(pow(i*T_IPO,2)+pow(te,2)+2*pow(tb,2))  + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos( ((2*M_PI)/tb) * (i*T_IPO-tv))))));
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
        pathArray.push_back(start_angle);
    }
    return true;
}


