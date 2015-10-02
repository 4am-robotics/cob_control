
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
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This module contains the implementation of all interface types.
 *
 ****************************************************************/
#define RAMP_PROFILE 1
#define SINOID_PROFILE 2
#include "cob_cartesian_controller/trajectory_profile_generator/trajactory_profile_generator.h"

/* BEGIN TrajectoryProfileBase *****************************************************************************************/

TrajectoryProfileBase* TrajectoryProfileBuilder::createProfile(const cob_cartesian_controller::CartesianActionStruct& params)
{
    TrajectoryProfileBase* ib = NULL;
    switch(params.profile.profile_type)
    {
        case RAMP_PROFILE:
            ib = new TrajectoryProfileRamp(params);
            break;
        case SINOID_PROFILE:
            ib = new TrajectoryProfileSinoid(params);
            break;
        default:
            ROS_ERROR("Unknown Profile");
            break;
    }

    return ib;
}
/* END TrajectoryProfileBase *******************************************************************************************/

/* BEGIN TrajectoryProfileRamp ********************************************************************************************/
inline void TrajectoryProfileRamp::getProfileTimings(double Se_max, double accl, double vel)
{
        int steps_te, steps_tv, steps_tb = 0;
        double tv, tb, te = 0.0;

        // Calculate the Ramp Profile Parameters
        if (vel > sqrt(std::fabs(Se_max) * accl))
        {
            vel = sqrt(std::fabs(Se_max) * accl);
        }
        tb = vel / accl;
        te = (std::fabs(Se_max) / vel) + tb;
        tv = te - tb;

        // Interpolationsteps for every timesequence
        steps_tb = trunc(tb      / params_.profile.t_ipo);
        steps_tv = trunc((tv-tb) / params_.profile.t_ipo);
        steps_te = trunc((te-tv) / params_.profile.t_ipo);

        // Reconfigure timings wtih t_ipo_
        pt_.tb = steps_tb * params_.profile.t_ipo;
        pt_.tv = (steps_tb + steps_tv) * params_.profile.t_ipo;
        pt_.te = (steps_tb + steps_tv + steps_te) * params_.profile.t_ipo;
        pt_.steps_tb = steps_tb;
        pt_.steps_tv = steps_tv;
        pt_.steps_te = steps_te;
}

inline bool TrajectoryProfileRamp::generatePath(cob_cartesian_controller::PathArray &pa)
{
    std::vector<double> array;
    double accl = params_.profile.accl;

    double direction = pa.getSe()/std::fabs(pa.getSe());

    if(pa.getCalcTe())
    {
        ROS_INFO_STREAM("Longest Se.. Vel_max: " << params_.profile.vel << "   accl_max: " << params_.profile.accl << "   tb: " << pt_.tb << "   tv: " << pt_.tv << "   te: " << pt_.te);

        int i = 1;
        // Calculate the ramp profile path
        // 0 <= t <= tb
        for(; i <= pt_.steps_tb ; i++)
        {
            array.push_back(pa.getStartValue() + direction * (0.5*params_.profile.accl*pow((params_.profile.t_ipo*i),2)));
        }

        // tb <= t <= tv
        for(; i <= (pt_.steps_tb + pt_.steps_tv) ; i++)
        {
            array.push_back(pa.getStartValue() + direction * (params_.profile.vel*(params_.profile.t_ipo*i) - 0.5 * pow(params_.profile.vel,2)/accl)  );
        }

        // tv <= t <= te
        for(; i <= (pt_.steps_tv + pt_.steps_tb + pt_.steps_te + 1) ; i++)
        {
            array.push_back(pa.getStartValue() + direction *(params_.profile.vel * (pt_.tv) - 0.5*accl* pow(pt_.te-(i*params_.profile.t_ipo),2)));
        }
    }
    else
    {
        if(std::fabs(pa.getSe()) > 0.001)
        {
            double vel_max;
            double accl_max = params_.profile.accl;
            double tb = pt_.tb;
            double tv = pt_.tv;
            double te = pt_.te;

            double steps_tb, steps_te, steps_tv;
            // Calculate the Profile Timings
            // Reconfigure accl_max and Velmax


//            accl_max = std::fabs(pa.getSe()) / pow(0.5 * pt_.te, 2);
            vel_max = accl_max * pt_.te / 2 - sqrt((pow(accl_max,2) * pow(pt_.te,2)/4) - std::fabs(pa.getSe()) * accl_max);

            tb = vel_max/accl_max;
            tv = pt_.te - tb;
            te = tv + tb;

            // Interpolationsteps for every timesequence
            steps_tb = trunc(tb      / params_.profile.t_ipo);
            steps_tv = trunc((tv-tb) / params_.profile.t_ipo);
            steps_te = trunc((te-tv) / params_.profile.t_ipo);

            // Reconfigure timings wtih t_ipo_
            tb = steps_tb * params_.profile.t_ipo;
            tv = (steps_tb + steps_tv) * params_.profile.t_ipo;
            te = (steps_tb + steps_tv + steps_te) * params_.profile.t_ipo;

            ROS_INFO_STREAM("Vel_max: " << vel_max << "   accl_max: " << accl_max << "   tb: " << tb << "   tv: " << tv << "   te: " << te);

            if(isnan(tb)!=0 || isnan(tv) !=0 || isnan(te) != 0 || isnan(accl_max) != 0 || isnan(vel_max) != 0) //NaN
            {
                ROS_WARN("Error while calculating the optimal velocity profile! \n Using the given parameters which may cause a longer operation time.");
                tb = params_.profile.vel / params_.profile.accl;
                tv = pt_.te - tb;
                te = tv + tb;
                steps_tb = trunc(tb      / params_.profile.t_ipo);
                steps_tv = trunc((tv-tb) / params_.profile.t_ipo);
                steps_te = trunc((te-tv) / params_.profile.t_ipo);
                accl_max = params_.profile.accl;
                vel_max  = params_.profile.vel;
            }

            // Calculate the ramp profile path
            int i = 1;
            // 0 <= t <= tb
            for(; i <= steps_tb ; i++)
            {
                array.push_back(pa.getStartValue() + direction * (0.5*accl_max*pow((params_.profile.t_ipo*i),2)));
            }
            // tb <= t <= tv
            for(; i <= (steps_tb + steps_tv); i++)
            {
                array.push_back(pa.getStartValue() + direction *(vel_max*(params_.profile.t_ipo*i) - 0.5*pow(vel_max,2)/accl_max));
            }
            // tv <= t <= te
            for(; i <= (steps_tv + steps_tb + steps_te + 1); i++)
            {
                array.push_back(pa.getStartValue() + direction * (vel_max * (te-tb) - 0.5*accl_max* pow(te-(i*params_.profile.t_ipo),2)));
            }
        }
        else
        {
            ROS_INFO_STREAM("Se = 0");

            array.push_back(pa.getStartValue());
        }
    }
    pa.setArray(array);
    return true;
}

inline bool TrajectoryProfileRamp::calculateProfile(std::vector<double> path_matrix[4],
                                                    const double Se, const double Se_roll, const double Se_pitch, const double Se_yaw,
                                                    geometry_msgs::Pose start)
{
    double accl = params_.profile.accl;
    double vel  = params_.profile.vel;
    unsigned int max_steps;

    //Convert to RPY
    double roll_start, pitch_start, yaw_start;
    tf::Quaternion q;
    tf::quaternionMsgToTF(start.orientation, q);
    tf::Matrix3x3(q).getRPY(roll_start, pitch_start, yaw_start);

    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;

    cob_cartesian_controller::PathArray lin(0, Se, 0, linear_path);
    cob_cartesian_controller::PathArray roll(1, Se_roll, roll_start, roll_path);
    cob_cartesian_controller::PathArray pitch(2, Se_pitch, pitch_start, pitch_path);
    cob_cartesian_controller::PathArray yaw(3, Se_yaw, yaw_start, yaw_path);
    cob_cartesian_controller::PathMatrix pm(lin,roll,pitch,yaw);

    std::vector<cob_cartesian_controller::PathArray> sortedMatrix = pm.getSortedMatrix();   // Sort the Matrix from the largest Se (0) to the smallest one (3)

    ROS_INFO("Got sorted_matrix.");

    this->getProfileTimings(std::fabs(sortedMatrix[0].getSe()), accl, vel);  // Calculate the velocity profile timings with respect to the largest Se

    ROS_INFO("Got profil timings.");

    // Calculate the paths
    for(int i=0; i<sortedMatrix.size(); i++)
    {
        this->generatePath(sortedMatrix[i]);
    }

    ROS_INFO("GeneratePath success.");

    // Resize the path vectors
    for(int i = 0; i < sortedMatrix.size(); i++)
    {
        max_steps = std::max((unsigned int)sortedMatrix[i].getArray().size() , max_steps);
    }

    ROS_INFO("max_steps success.");

    // Re-adjust the Matrix to its originally form. Index 0 to 3
    std::vector<double> temp_array;
    cob_cartesian_controller::PathArray temp(0, 0, 0.0, temp_array);

    for(int j = 0; j < sortedMatrix.size(); j++)
    {
        for(int i = 0; i < sortedMatrix.size()-1; i++)
        {
            if(sortedMatrix[i].getIdx() > sortedMatrix[i+1].getIdx())
            {
                temp = sortedMatrix[i];
                sortedMatrix[i] = sortedMatrix[i+1];
                sortedMatrix[i+1] = temp;
            }
        }
    }

    ROS_INFO("IndexSort success.");

    for(int i = 0; i < sortedMatrix.size(); i++)
    {
        path_matrix[i] = sortedMatrix[i].getArray();
    }

    ROS_INFO("Fill path_matrix success.");

    for(int i = 0; i < sortedMatrix.size(); i++)
    {
        if(path_matrix[i].size() < max_steps)
        {
            path_matrix[i].resize(max_steps, path_matrix[i].back());
        }
    }

    ROS_INFO("Readjust size success.");
    return true;
}


/* END TrajectoryProfileRamp **********************************************************************************************/







/* BEGIN TrajectoryProfileSinoid ****************************************************************************************/
inline void TrajectoryProfileSinoid::getProfileTimings(double Se_max, double accl, double vel)
{
        int steps_te, steps_tv, steps_tb = 0;
        double tv, tb, te = 0.0;

        // Calculate the Ramp Profile Parameters
        if (vel > sqrt(std::fabs(Se_max) * accl / 2))
        {
            vel = sqrt(std::fabs(Se_max) * accl / 2);
        }
        tb = 2 * vel / accl;
        te = (std::fabs(Se_max) / vel) + tb;
        tv = te - tb;

        // Interpolationsteps for every timesequence
        steps_tb = (double)tb      / params_.profile.t_ipo;
        steps_tv = (double)(tv-tb) / params_.profile.t_ipo;
        steps_te = (double)(te-tv) / params_.profile.t_ipo;

        // Reconfigure timings wtih t_ipo_
        pt_.tb = steps_tb * params_.profile.t_ipo;
        pt_.tv = (steps_tb + steps_tv) * params_.profile.t_ipo;
        pt_.te = (steps_tb + steps_tv + steps_te) * params_.profile.t_ipo;
        pt_.steps_tb = steps_tb;
        pt_.steps_tv = steps_tv;
        pt_.steps_te = steps_te;
}

inline bool TrajectoryProfileSinoid::generatePath(cob_cartesian_controller::PathArray &pa)
{
    std::vector<double> array;
    double accl = params_.profile.accl;

    if(pa.getCalcTe())
    {
        ROS_INFO_STREAM("Longest Se.. Vel_max: " << params_.profile.vel << "   accl_max: " << params_.profile.accl << "   tb: " << pt_.tb << "   tv: " << pt_.tv << "   te: " << pt_.te);

        int i = 1;
        // Calculate the sinoid profile path
        for(; i <= pt_.steps_tb ; i++)
        {
            array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*( params_.profile.accl*(0.25*pow(i*params_.profile.t_ipo,2) + pow(pt_.tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/pt_.tb * (i*params_.profile.t_ipo))-1))));
        }
        // tb <= t <= tv
        for(; i <= (pt_.steps_tb + pt_.steps_tv) ; i++)
        {
            array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*( params_.profile.vel*(i*params_.profile.t_ipo-0.5*pt_.tb)));
        }
        // tv <= t <= te
        for(; i <= (pt_.steps_tv + pt_.steps_tb + pt_.steps_te + 1) ; i++)
        {
            array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*( 0.5 * params_.profile.accl *(pt_.te*(i*params_.profile.t_ipo + pt_.tb) - 0.5*(pow(i*params_.profile.t_ipo,2)+pow(pt_.te,2)+2*pow(pt_.tb,2)) + (pow(pt_.tb,2)/(4*pow(M_PI,2))) * (1-cos(((2*M_PI)/pt_.tb) * (i*params_.profile.t_ipo-pt_.tv))))));
        }
    }
    else
    {
        if(std::fabs(pa.getSe()) > 0.001)
        {
            double vel_max;
            double accl_max = params_.profile.accl;
            double tb,te,tv;
            double steps_tb, steps_te, steps_tv;

            // Calculate the Profile Timings
            // Reconfigure accl_max and Velmax

            accl_max = 8 * std::fabs(pa.getSe())/pow(pt_.te, 2);
            vel_max = accl_max * pt_.te / 4 - sqrt((pow(accl_max,2)*pow(pt_.te,2)/16) - std::fabs(pa.getSe()) * accl_max/2 );

            tb = 2 * vel_max / accl_max;
            tv = pt_.te - tb;
            te = tv + tb;

            // Interpolationsteps for every timesequence
            steps_tb = trunc(tb      / params_.profile.t_ipo);
            steps_tv = trunc((tv-tb) / params_.profile.t_ipo);
            steps_te = trunc((te-tv) / params_.profile.t_ipo);

            if(isnan(tb)!=0 || isnan(tv) !=0 || isnan(te) != 0 || isnan(accl_max) != 0 || isnan(vel_max) != 0) //NaN
            {
                ROS_WARN("Error while calculating the optimal velocity profile! \n Using the given parameters which may cause a longer operation time.");
                tb = 2 * params_.profile.vel / params_.profile.accl;
                tv = pt_.te - tb;
                te = tv + tb;
                steps_tb = trunc(tb      / params_.profile.t_ipo);
                steps_tv = trunc((tv-tb) / params_.profile.t_ipo);
                steps_te = trunc((te-tv) / params_.profile.t_ipo);
                accl_max = params_.profile.accl;
                vel_max  = params_.profile.vel;
            }

            ROS_INFO_STREAM("Vel_max: " << vel_max << "   accl_max: " << accl_max << "   tb: " << tb << "   tv: " << tv << "   te: " << te);
            int i = 1;
            // Calculate the sinoid profile path
            for(; i <= steps_tb ; i++)
            {
                array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*(accl_max*(0.25*pow(i*params_.profile.t_ipo,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*params_.profile.t_ipo))-1))));
            }
            // tb <= t <= tv
            for(; i <= (steps_tb + steps_tv) ; i++)
            {
                array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*( vel_max*(i*params_.profile.t_ipo-0.5*tb)));
            }
            // tv <= t <= te
            for(; i <= (steps_tv + steps_tb + steps_te + 1) ; i++)
            {
                array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*( 0.5 * accl_max *(te*(i*params_.profile.t_ipo + tb) - 0.5*(pow(i*params_.profile.t_ipo,2)+pow(te,2)+2*pow(tb,2)) + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos(((2*M_PI)/tb) * (i*params_.profile.t_ipo-tv))))));
            }
        }
        else
        {
            ROS_INFO_STREAM("Se = 0");
            array.push_back(0);
        }
    }

    pa.setArray(array);
    return true;
}

inline bool TrajectoryProfileSinoid::calculateProfile(std::vector<double> path_matrix[4],
                                                    const double Se, const double Se_roll, const double Se_pitch, const double Se_yaw,
                                                    geometry_msgs::Pose start)
{
    double accl = params_.profile.accl;
    double vel  = params_.profile.vel;
    unsigned int max_steps;

    //Convert to RPY
    double roll_start, pitch_start, yaw_start;
    tf::Quaternion q;
    tf::quaternionMsgToTF(start.orientation, q);
    tf::Matrix3x3(q).getRPY(roll_start, pitch_start, yaw_start);

    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;

    cob_cartesian_controller::PathArray lin(0, Se, 0, linear_path);
    cob_cartesian_controller::PathArray roll(1, Se_roll, roll_start, roll_path);
    cob_cartesian_controller::PathArray pitch(2, Se_pitch, pitch_start, pitch_path);
    cob_cartesian_controller::PathArray yaw(3, Se_yaw, yaw_start, yaw_path);
    cob_cartesian_controller::PathMatrix pm(lin,roll,pitch,yaw);

    std::vector<cob_cartesian_controller::PathArray> sortedMatrix = pm.getSortedMatrix();   // Sort the Matrix from the largest Se (0) to the smallest one (3)
    this->getProfileTimings(std::fabs(sortedMatrix[0].getSe()), accl, vel);  // Calculate the velocity profile timings with respect to the largest Se

    // Calculate the paths
    for(int i=0; i<sortedMatrix.size(); i++)
    {
        this->generatePath(sortedMatrix[i]);
    }

    // Resize the path vectors
    for(int i = 0; i < sortedMatrix.size(); i++)
    {
        max_steps = std::max((unsigned int)sortedMatrix[i].getArray().size() , max_steps);
    }

    // Re-adjust the Matrix to its originally form. Index 0 to 3
    std::vector<double> temp_array;
    cob_cartesian_controller::PathArray temp(0, 0, 0.0, temp_array);
    for(int j = 0; j<sortedMatrix.size(); j++)
    {
        for(int i = 0; i < sortedMatrix.size()-1; i++)
        {
            if(sortedMatrix[i].getIdx() > sortedMatrix[i+1].getIdx())
            {
                temp = sortedMatrix[i];
                sortedMatrix[i] = sortedMatrix[i+1];
                sortedMatrix[i+1] = temp;
            }
        }
    }

    for(int i = 0; i < sortedMatrix.size(); i++)
    {
        path_matrix[i] = sortedMatrix[i].getArray();
    }

    for(int i = 0; i < sortedMatrix.size(); i++)
    {
        if(path_matrix[i].size() < max_steps)
        {
            path_matrix[i].resize(max_steps, path_matrix[i].back());
        }
    }
    return true;
}
/* END TrajectoryProfileSinoid ******************************************************************************************/
