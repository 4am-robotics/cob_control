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
 * \date Date of creation: December, 2015
 *
 * \brief
 *   Base class for trajectory_profile_generator.
 *
 ****************************************************************/

#ifndef COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_BASE_H
#define COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_BASE_H

#include <ros/ros.h>
#include <cob_cartesian_controller/cartesian_controller_data_types.h>
#include <cob_cartesian_controller/cartesian_controller_utils.h>


#define MIN_VELOCITY_THRESHOLD 0.001

class TrajectoryProfileBase
{
public:
    TrajectoryProfileBase(const cob_cartesian_controller::CartesianActionStruct& params)
    :    params_(params)
    {}

    virtual ~TrajectoryProfileBase()
    {}

    virtual cob_cartesian_controller::ProfileTimings getProfileTimings(double Se, double te, double accl, double vel, bool calcMaxTe) = 0;

    virtual bool calculateProfile(std::vector<double>* path_matrix,
                                  const double Se_lin, const double Se_rot,
                                  geometry_msgs::Pose start) = 0;

    virtual std::vector<double> getTrajectory(double se, double accl, double vel, double t_ipo,
                                              double steps_tb, double steps_tv, double steps_te, double tb, double tv, double te) = 0;
private:
    virtual bool generatePath(cob_cartesian_controller::PathArray& pa) = 0;

protected:
    const cob_cartesian_controller::CartesianActionStruct& params_;
};

#endif  // COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_BASE_H
