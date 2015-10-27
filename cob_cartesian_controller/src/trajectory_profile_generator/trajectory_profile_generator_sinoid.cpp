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
inline cob_cartesian_controller::ProfileTimings TrajectoryProfileSinoid::getProfileTimings(double te_max, double accl, double vel)
{
        cob_cartesian_controller::ProfileTimings pt;
        int steps_te, steps_tv, steps_tb = 0;
        double tv, tb, te = 0.0;

        tb = 2 * vel / accl;
        te = te_max + tb;
        tv = te - tb;

        // Interpolationsteps for every timesequence
        steps_tb = round(tb      / params_.profile.t_ipo);
        steps_tv = round((tv-tb) / params_.profile.t_ipo);
        steps_te = round((te-tv) / params_.profile.t_ipo);

        // Reconfigure timings wtih t_ipo_
        pt.tb = steps_tb * params_.profile.t_ipo;
        pt.tv = (steps_tb + steps_tv) * params_.profile.t_ipo;
        pt.te = (steps_tb + steps_tv + steps_te) * params_.profile.t_ipo;
        pt.steps_tb = steps_tb;
        pt.steps_tv = steps_tv;
        pt.steps_te = steps_te;

        return pt;
}

inline bool TrajectoryProfileSinoid::generatePath(cob_cartesian_controller::PathArray &pa)
{
    std::vector<double> array;
    cob_cartesian_controller::ProfileTimings pt;
    double accl_max = params_.profile.accl;
    double vel_max = params_.profile.vel;

    if(std::fabs(pa.Se_) > 0.001)
    {
        if(pa.calcTe_)
        {
            pt = pt_max_;
        }
        else
        {
            // Calculate the Profile Timings
            vel_max = accl_max * pt_max_.te / 4 - sqrt((pow(accl_max,2)*pow(pt_max_.te,2)/16) - std::fabs(pa.Se_) * accl_max/2 );

            pt = getProfileTimings(pt_max_.te, accl_max, vel_max);
        }

        array = getTrajectory(pa.start_value_ , pa.Se_, accl_max, vel_max, params_.profile.t_ipo, pt.steps_tb, pt.steps_tv, pt.steps_te, pt.tb, pt.tv, pt.te);

        switch(pa.idx_)
        {
            case 0:
            {
                ROS_INFO_STREAM("Linear-Path: "  << pa.Se_ << std::setw(10) << std::left << " Velocity: " << vel_max << std::setw(10) <<" Acceleration: " << accl_max);
                break;
            }
            case 1:
            {
                ROS_INFO_STREAM("Roll-Path: " << pa.Se_ << std::setw(10) << std::left << " Velocity: " << vel_max << std::setw(10) << " Acceleration: " << accl_max);
                break;
            }
            case 2:
            {
                ROS_INFO_STREAM("Pitch-Path: " << pa.Se_ << std::setw(10) << std::left << " Velocity: " << vel_max << std::setw(10) <<" Acceleration: " << accl_max);
                break;
            }
            case 3:
            {
                ROS_INFO_STREAM("Yaw-Path: " << pa.Se_ << std::setw(10) << std::left <<" Velocity: " << vel_max << std::setw(10) <<" Acceleration: " << accl_max);
                break;
            }
        }
    }
    else
    {
        switch(pa.idx_)
        {
            case 0:
            {
                ROS_INFO_STREAM("Linear-Path: " << pa.Se_ << " Velocity: 0" << "Acceleration: 0");
                break;
            }
            case 1:
            {
                ROS_INFO_STREAM("Roll-Path: " << pa.Se_ << " Velocity: 0" << "Acceleration: 0");
                break;
            }
            case 2:
            {
                ROS_INFO_STREAM("Pitch-Path: " << pa.Se_ << " Velocity: 0" << "Acceleration: 0");
                break;
            }
            case 3:
            {
                ROS_INFO_STREAM("Yaw-Path: " << pa.Se_ << " Velocity: 0" << "Acceleration: 0");
                break;
            }
        }
        array.push_back(pa.start_value_);
    }

    pa.array_ = array;
    return true;
}

inline std::vector<double> TrajectoryProfileSinoid::getTrajectory(double start_value, double se, double accl, double vel, double t_ipo, double steps_tb, double steps_tv, double steps_te, double tb, double tv, double te)
{
    std::vector<double> array;
    unsigned int i = 1;
    double direction = se/std::fabs(se);
    //Calculate the sinoid profile path
    for(; i <= steps_tb ; i++)
    {
        array.push_back(start_value + direction * (accl*(0.25*pow(i*t_ipo,2) + pow(tb,2)/(8*pow(M_PI,2)) *(cos(2*M_PI/tb * (i*t_ipo))-1))));
    }
    // tb <= t <= tv
    for(; i <= (steps_tb + steps_tv) ; i++)
    {
        array.push_back(start_value + direction * ( vel*(i*t_ipo-0.5*tb)));
    }
    // tv <= t <= te
    for(; i <= (steps_tv + steps_tb + steps_te + 1) ; i++)
    {
        array.push_back(start_value + direction * (0.5 * accl *(te*(i*t_ipo + tb) - 0.5*(pow(i*t_ipo,2)+pow(te,2)+2*pow(tb,2)) + (pow(tb,2)/(4*pow(M_PI,2))) * (1-cos(((2*M_PI)/tb) * (i*t_ipo-tv))))));
    }
    return array;
}

inline bool TrajectoryProfileSinoid::calculateProfile(std::vector<double> path_matrix[4],
                                                    const double Se, const double Se_roll, const double Se_pitch, const double Se_yaw,
                                                    geometry_msgs::Pose start)
{
    double accl = params_.profile.accl;
    double vel  = params_.profile.vel;
    unsigned int max_steps = 0;

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

    // Calculate the Sinoid Profile Parameters
    if (vel > sqrt(std::fabs(sortedMatrix[0].Se_) * accl / 2))
    {
        vel = sqrt(std::fabs(sortedMatrix[0].Se_) * accl / 2);
    }
    pt_max_ = this->getProfileTimings((std::fabs(sortedMatrix[0].Se_) / vel), accl, vel);  // Calculate the velocity profile timings with respect to the largest Se

    // Calculate the paths
    for(int i=0; i<sortedMatrix.size(); i++)
    {
        this->generatePath(sortedMatrix[i]);
    }

    // Resize the path vectors
    for(int i = 0; i < sortedMatrix.size(); i++)
    {
        max_steps = std::max((unsigned int)sortedMatrix[i].array_.size() , max_steps);
    }

    // Re-adjust the Matrix to its originally form. Index 0 to 3
    std::vector<double> temp_array;
    cob_cartesian_controller::PathArray temp(0, 0, 0.0, temp_array);
    for(int j = 0; j<sortedMatrix.size(); j++)
    {
        for(int i = 0; i < sortedMatrix.size()-1; i++)
        {
            if(sortedMatrix[i].idx_ > sortedMatrix[i+1].idx_)
            {
                temp = sortedMatrix[i];
                sortedMatrix[i] = sortedMatrix[i+1];
                sortedMatrix[i+1] = temp;
            }
        }
    }

    for(int i = 0; i < sortedMatrix.size(); i++)
    {
        path_matrix[i] = sortedMatrix[i].array_;
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

