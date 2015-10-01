
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
        pt_.max_steps = steps_tb + steps_tv + steps_te;
}

inline void TrajectoryProfileRamp::generatePath(cob_cartesian_controller::PathArray &pa)
{
    std::vector<double> array;
    double accl = params_.profile.accl;

    if(pa.getCalcTe())
    {
        int i = 0;
        // Calculate the ramp profile path
        // 0 <= t <= tb
        for(; i <= pt_.steps_tb ; i++)
        {
            array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*(0.5*accl*pow((params_.profile.t_ipo*i),2)));
        }

        // tb <= t <= tv
        for(; i <= (pt_.steps_tb + pt_.steps_tv) ; i++)
        {
            array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*(params_.profile.vel*(params_.profile.t_ipo*i)-0.5*pow(params_.profile.vel,2)/accl));
        }

        // tv <= t <= te
        for(; i <= (pt_.steps_tv + pt_.steps_tb + pt_.steps_te) ; i++)
        {
            array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*(params_.profile.vel * (pt_.te-pt_.tb) - 0.5*accl* pow(pt_.te-(i*params_.profile.t_ipo),2)));
        }
    }
    else
    {
        if(std::fabs(pa.getSe()) > 0.001)
        {
            double vel_max;
            double accl_max = params_.profile.accl;
            // Calculate the Profile Timings
            // Reconfigure accl_max and Velmax
            while(pt_.te < 2 * sqrt(std::fabs(pa.getSe())/accl_max))
            {
                accl_max+=0.001;
            }
            vel_max = accl_max * pt_.te / 2 - sqrt((pow(accl_max,2)*pow(pt_.te,2)/4) - std::fabs(pa.getSe()) * accl_max );

            // Calculate the ramp profile path
            int i=0;
            // 0 <= t <= tb
            for(; i <= pt_.steps_tb ; i++)
            {
                array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*(0.5*accl_max*pow((params_.profile.t_ipo*i),2)));
            }
            // tb <= t <= tv
            for(;i<=(pt_.steps_tb + pt_.steps_tv);i++)
            {
                array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*(vel_max*(params_.profile.t_ipo*i)-0.5*pow(vel_max,2)/accl_max));
            }
            // tv <= t <= te
            for(; i <= (pt_.steps_tv + pt_.steps_tb + pt_.steps_te);i++)
            {
                array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*(vel_max * (pt_.te-pt_.tb) - 0.5*accl_max* pow(pt_.te-(i*params_.profile.t_ipo),2)));
            }
        }
        else
        {
            array.push_back(0);
        }
    }


    pa.setArray(array);



}

inline bool TrajectoryProfileRamp::calculateProfile(std::vector<double> path_matrix[4],
                                                    const double Se, const double Se_roll, const double Se_pitch, const double Se_yaw,
                                                    geometry_msgs::Pose start)
{
    double accl = params_.profile.accl;
    double vel  = params_.profile.vel;

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

    getProfileTimings(sortedMatrix[0].getSe(), accl, vel);  // Calculate the velocity profile timings with respect to the largest Se

    // Calculate the paths
    for(int i=0; i<4; i++)
    {
        this->generatePath(sortedMatrix[i]);
    }

    // Resize the path vectors
    for(int i=0; i < 4; i++)
    {
        this->pt_.max_steps = std::max((unsigned int)sortedMatrix[i].getArray().size() , this->pt_.max_steps);
    }

    // Re-adjust the Matrix to its originally form. Index 0 to 3
    std::vector<double> temp_array;
    cob_cartesian_controller::PathArray temp(0, 0, 0.0, temp_array);
    for(int j = 0; j<4; j++)
    {
        for(int i = 0; i<3; i++)
        {
            if(sortedMatrix[i].getIdx() > sortedMatrix[i+1].getIdx())
            {
                temp = sortedMatrix[i];
                sortedMatrix[i] = sortedMatrix[i+1];
                sortedMatrix[i+1] = temp;
            }
        }
    }

    path_matrix[0] = sortedMatrix[0].getArray();
    path_matrix[1] = sortedMatrix[1].getArray();
    path_matrix[2] = sortedMatrix[2].getArray();
    path_matrix[3] = sortedMatrix[3].getArray();

    for(int i = 0; i < 4; i++)
    {
        if(path_matrix[i].size() < pt_.max_steps)
        {
            path_matrix[i].resize(pt_.max_steps, path_matrix[i].back());
        }
    }

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
        pt_.max_steps = steps_tb + steps_tv + steps_te;
}

inline void TrajectoryProfileSinoid::generatePath(cob_cartesian_controller::PathArray &pa)
{
    std::vector<double> array;
    double accl = params_.profile.accl;

    if(pa.getCalcTe())
    {
        int i = 0;
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
        for(; i < (pt_.steps_tv + pt_.steps_tb + pt_.steps_te) ; i++)
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
            // Calculate the Profile Timings
            // Reconfigure accl_max and Velmax
            while(pt_.te < sqrt(std::fabs(pa.getSe()) * 8/accl_max))
            {
                accl_max += 0.001;
            }
            vel_max = accl_max * pt_.te / 4 - sqrt((pow(accl_max,2)*pow(pt_.te,2)/16) - std::fabs(pa.getSe()) * accl_max/2 );

            int i = 0;
            // Calculate the sinoid profile path
            for(; i <= pt_.steps_tb ; i++)
            {
                array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*(accl_max*(0.25*pow(i*params_.profile.t_ipo,2) + pow(pt_.tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/pt_.tb * (i*params_.profile.t_ipo))-1))));
            }
            // tb <= t <= tv
            for(; i <= (pt_.steps_tb + pt_.steps_tv) ; i++)
            {
                array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*( vel_max*(i*params_.profile.t_ipo-0.5*pt_.tb)));
            }
            // tv <= t <= te
            for(; i < (pt_.steps_tv + pt_.steps_tb + pt_.steps_te) ; i++)
            {
                array.push_back(pa.getStartValue() + pa.getSe()/std::fabs(pa.getSe())*( 0.5 * accl_max *(pt_.te*(i*params_.profile.t_ipo + pt_.tb) - 0.5*(pow(i*params_.profile.t_ipo,2)+pow(pt_.te,2)+2*pow(pt_.tb,2)) + (pow(pt_.tb,2)/(4*pow(M_PI,2))) * (1-cos(((2*M_PI)/pt_.tb) * (i*params_.profile.t_ipo-pt_.tv))))));
            }
        }
        else
        {
            array.push_back(0);
        }
    }

    pa.setArray(array);
}

inline bool TrajectoryProfileSinoid::calculateProfile(std::vector<double> path_matrix[4],
                                                    const double Se, const double Se_roll, const double Se_pitch, const double Se_yaw,
                                                    geometry_msgs::Pose start)
{
    double accl = params_.profile.accl;
    double vel  = params_.profile.vel;

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

    getProfileTimings(sortedMatrix[0].getSe(), accl, vel);  // Calculate the velocity profile timings with respect to the largest Se

    // Calculate the paths
    for(int i=0; i<4; i++)
    {
        this->generatePath(sortedMatrix[i]);
    }

    // Resize the path vectors
    for(int i=0; i < 4; i++)
    {
        this->pt_.max_steps = std::max((unsigned int)sortedMatrix[i].getArray().size() , this->pt_.max_steps);
    }

    // Re-adjust the Matrix to its originally form. Index 0 to 3
    std::vector<double> temp_array;
    cob_cartesian_controller::PathArray temp(0, 0, 0.0, temp_array);
    for(int j = 0; j<4; j++)
    {
        for(int i = 0; i<3; i++)
        {
            if(sortedMatrix[i].getIdx() > sortedMatrix[i+1].getIdx())
            {
                temp = sortedMatrix[i];
                sortedMatrix[i] = sortedMatrix[i+1];
                sortedMatrix[i+1] = temp;
            }
        }
    }

    path_matrix[0] = sortedMatrix[0].getArray();
    path_matrix[1] = sortedMatrix[1].getArray();
    path_matrix[2] = sortedMatrix[2].getArray();
    path_matrix[3] = sortedMatrix[3].getArray();

    for(int i = 0; i < 4; i++)
    {
        if(path_matrix[i].size() < pt_.max_steps)
        {
            path_matrix[i].resize(pt_.max_steps, path_matrix[i].back());
        }
    }

    return true;
}
/* END TrajectoryProfileSinoid ******************************************************************************************/
