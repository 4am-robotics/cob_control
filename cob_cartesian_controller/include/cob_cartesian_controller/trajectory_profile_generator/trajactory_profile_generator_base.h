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
 *   This header contains the interface description of all available
 *   hardware interface types (position/velocity).
 *
 ****************************************************************/
#ifndef HARDWARE_INTERFACE_TYPE_BASE_H_
#define HARDWARE_INTERFACE_TYPE_BASE_H_

#include "ros/ros.h"
#include "cob_cartesian_controller/cartesian_controller_data_types.h"
#include <cob_cartesian_controller/cartesian_controller_utils.h>

/// Base class for hardware interfaces types.
class TrajectoryProfileBase
{
    public:
        TrajectoryProfileBase(const cob_cartesian_controller::CartesianActionStruct& params):
            params_(params)
        {vel_max_=0;}

        virtual ~TrajectoryProfileBase() {}

        virtual cob_cartesian_controller::ProfileTimings getMaxProfileTimings(double Se_max, double accl, double vel) = 0;
        virtual bool calculateProfile(std::vector<double> path_matrix[4],
                                      double Se, double Se_roll, double Se_pitch, double Se_yaw,
                                      geometry_msgs::Pose start) = 0;
        virtual std::vector<double> getTrajectory(double start_value, double se,
                                                  double accl, double vel, double t_ipo,
                                                  double steps_tb, double steps_tv, double steps_te, double tb, double tv, double te) = 0;
    private:
        virtual bool generatePath(cob_cartesian_controller::PathArray &pa) = 0;
        virtual cob_cartesian_controller::ProfileTimings getProfileTimings(double te_max, double accl, double vel) = 0;


    protected:
        const cob_cartesian_controller::CartesianActionStruct &params_;
        double vel_max_;
};

#endif /* HARDWARE_INTERFACE_TYPE_BASE_H_ */
