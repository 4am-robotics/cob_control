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
 *   Class implementing the Sinoid velocity profile generator.
 *
 ****************************************************************/

#ifndef COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_SINOID_H
#define COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_SINOID_H

#include <vector>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_base.h>

/* BEGIN TrajectoryProfileSinoid ****************************************************************************************/
class TrajectoryProfileSinoid : public TrajectoryProfileBase
{
public:
    explicit TrajectoryProfileSinoid(const cob_cartesian_controller::CartesianActionStruct& params)
    :   TrajectoryProfileBase(params)
    {}

    ~TrajectoryProfileSinoid()
    {}

    virtual cob_cartesian_controller::ProfileTimings getProfileTimings(double Se, double te, double accl, double vel, bool calcMaxTe);
    virtual std::vector<double> getTrajectory(double se, double accl, double vel, double t_ipo,
                                              double steps_tb, double steps_tv, double steps_te, double tb, double tv, double te);
};
/* END TrajectoryProfileSinoid **********************************************************************************************/

#endif  // COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_SINOID_H
