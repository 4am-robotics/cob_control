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

#ifndef COB_CARTESIAN_CONTROLLER_PROFILE_GENERATOR_CIRC_H_
#define COB_CARTESIAN_CONTROLLER_PROFILE_GENERATOR_CIRC_H_

#include <ros/ros.h>
#include <cob_cartesian_controller/cartesian_controller_data_types.h>
#include <cob_cartesian_controller/cartesian_controller_utils.h>


class TrajectoryProfileGeneratorCirc
{
public:
    TrajectoryProfileGeneratorCirc(double update_rate)
    {
        update_rate_ = update_rate;
        t_ipo_ = 1.0/update_rate;
    }

    ~TrajectoryProfileGeneratorCirc()
    {}

    bool calculateProfile(std::vector<double>& path_array, const double Se, cob_cartesian_controller::ProfileStruct& profile);

private:
    bool calculateRampProfile(std::vector<double>& path_array, const double Se, cob_cartesian_controller::ProfileStruct& profile);
    bool calculateSinoidProfile(std::vector<double>& path_array, const double Se, cob_cartesian_controller::ProfileStruct& profile);

    double update_rate_;
    double t_ipo_;
};

#endif /* COB_CARTESIAN_CONTROLLER_PROFILE_GENERATOR_CIRC_H_ */
