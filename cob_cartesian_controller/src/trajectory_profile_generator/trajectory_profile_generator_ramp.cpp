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
inline cob_cartesian_controller::ProfileTimings TrajectoryProfileRamp::getProfileTimings(double te_max, double accl, double vel)
{
        cob_cartesian_controller::ProfileTimings pt;
        int steps_te, steps_tv, steps_tb = 0;
        double tv, tb, te = 0.0;

        tb = vel / accl;
//        te = (std::fabs(Se) / vel) + tb;
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

inline cob_cartesian_controller::ProfileTimings TrajectoryProfileRamp::getMaxProfileTimings(double Se_max, double accl, double vel)
{
    // Calculate the Ramp Profile Parameters
    if (vel > sqrt(std::fabs(Se_max) * accl))
    {
        vel = sqrt(std::fabs(Se_max) * accl);
    }

    return getProfileTimings((std::fabs(Se_max) / vel), accl, vel);
}

inline bool TrajectoryProfileRamp::generatePath(cob_cartesian_controller::PathArray &pa)
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
            vel_max = accl_max * pt_max_.te / 2 - sqrt((pow(accl_max,2) * pow(pt_max_.te,2)/4) - std::fabs(pa.Se_) * accl_max);
            pt = getProfileTimings(pt_max_.te, accl_max, vel_max);
        }

        array = getTrajectory(pa.start_value_, pa.Se_, accl_max, vel_max, params_.profile.t_ipo, pt.steps_tb, pt.steps_tv, pt.steps_te, pt.tb, pt.tv, pt.te);

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
        array.push_back(start_value + direction * (0.5*accl*pow((params_.profile.t_ipo*i),2)));
    }
    // tb <= t <= tv
    for(; i <= (steps_tb + steps_tv); i++)
    {
        array.push_back(start_value + direction *(vel*(params_.profile.t_ipo*i) - 0.5*pow(vel,2)/accl));
    }
    // tv <= t <= te
    for(; i <= (steps_tv + steps_tb + steps_te + 1); i++)
    {
        array.push_back(start_value + direction * (vel * (te-tb) - 0.5 * accl * pow(te-(i*params_.profile.t_ipo),2)));
    }

    return array;
}

inline bool TrajectoryProfileRamp::calculateProfile(std::vector<double> path_matrix[4],
                                                    const double Se, const double Se_roll, const double Se_pitch, const double Se_yaw,
                                                    geometry_msgs::Pose start)
{
    CartesianControllerUtils ccu;
    std::vector<double> linear_path, roll_path, pitch_path, yaw_path;
    std::vector<cob_cartesian_controller::PathArray> sortedMatrix;
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

    // Sort the Matrix from the largest Se (0) to the smallest one (3)
    sortedMatrix = pm.getSortedMatrix();

    // Get the profile timings from the longest path
    pt_max_ = getMaxProfileTimings(sortedMatrix[0].Se_, params_.profile.accl, params_.profile.vel);

    // Calculate the paths
    for(int i = 0; i < sortedMatrix.size(); i++)
    {
        generatePath(sortedMatrix[i]);
    }

    // Adjust the array length
    ccu.adjustArrayLength(sortedMatrix);

    // Sort the arrays in the following order lin(0), roll(1), pitch(2), yaw(3)
    ccu.sortMatrixByIdx(sortedMatrix);

    ccu.copyMatrix(path_matrix,sortedMatrix);

    return true;
}

/* END TrajectoryProfileRamp **********************************************************************************************/
