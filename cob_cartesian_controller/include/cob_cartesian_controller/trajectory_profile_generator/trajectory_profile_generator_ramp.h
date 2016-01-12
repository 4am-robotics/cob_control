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
 *   Class implementing the Ramp velocity profile generator.
 *
 ****************************************************************/

#ifndef COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_RAMP_H
#define COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_RAMP_H

#include <vector>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_base.h>

/* BEGIN TrajectoryProfileRamp ****************************************************************************************/
class TrajectoryProfileRamp: public TrajectoryProfileBase
{
public:
    explicit TrajectoryProfileRamp(const cob_cartesian_controller::CartesianActionStruct& params)
    :    TrajectoryProfileBase(params)
    {}

    ~TrajectoryProfileRamp()
    {}

    virtual bool getProfileTimings(double Se, double te, bool calcMaxTe, cob_cartesian_controller::ProfileTimings& pt);
    virtual std::vector<double> getTrajectory(double se, cob_cartesian_controller::ProfileTimings pt);
};
/* END TrajectoryProfileRamp **********************************************************************************************/

#endif  // COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_RAMP_H
