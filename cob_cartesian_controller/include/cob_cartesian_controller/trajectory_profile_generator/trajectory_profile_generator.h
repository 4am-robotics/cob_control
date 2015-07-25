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

#ifndef COB_CARTESIAN_CONTROLLER_PROFILE_GENERATOR_H_
#define COB_CARTESIAN_CONTROLLER_PROFILE_GENERATOR_H_

#include <ros/ros.h>
#include <cob_cartesian_controller/cartesian_controller_data_types.h>
#include <cob_cartesian_controller/cartesian_controller_utils.h>


class TrajectoryProfileGenerator
{
public:
    TrajectoryProfileGenerator(double update_rate)
    {
        update_rate_ = update_rate;
    }
    
    ~TrajectoryProfileGenerator()
    {}

    bool calculateProfile(std::vector<double>& path_array, double Se, double vel_max, double accl_max, unsigned int profile_type);

    bool calculateProfileForAngularMovements(std::vector<double>* path_matrix,
                                             double Se, double Se_roll, double Se_pitch, double Se_yaw,
                                             cob_cartesian_controller::MoveLinStruct& move_lin);

private:
    bool generatePath(std::vector<double>& path_rray, double T_IPO, double vel_max, double accel_max,
                      double Se_max, int steps_max, unsigned int profile);

    bool generatePathWithTe(std::vector<double> &path_array, double T_IPO, double te, double accl_max, double Se_max,
                            int steps_max, double start_angle, unsigned int profile);

    double update_rate_;
};

#endif /* COB_CARTESIAN_CONTROLLER_PROFILE_GENERATOR_H_ */
