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

#ifndef COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_H_
#define COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_H_

#include <ros/ros.h>

class TrajectoryProfileGenerator
{
public:
    TrajectoryProfileGenerator(double update_rate) {
        update_rate_ = update_rate;
    }
    ~TrajectoryProfileGenerator(){}

    void calculateProfile(std::vector<double> &pathArray, double Se, double VelMax, double AcclMax, std::string profile);

    void calculateProfileForAngularMovements(std::vector<double> *pathMatrix,
                                             double Se, double Se_roll, double Se_pitch, double Se_yaw,
                                             double start_angle_roll, double start_angle_pitch, double start_angle_yaw,
                                             double VelMax, double AcclMax, std::string profile, bool justRotate);
private:


    bool generatePath(std::vector<double> &pathArray,double T_IPO, double VelMax,
                      double AcclMax, double Se_max, int steps_max, std::string profile);


    bool generatePathWithTe(    std::vector<double> &pathArray, double T_IPO, double te, double AcclMax, double Se_max,
                                int steps_max, double start_angle, std::string profile);

    double update_rate_;
};



#endif /* COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_H_ */
