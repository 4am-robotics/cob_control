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
#ifndef COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_RAMP_H_
#define COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_RAMP_H_


#include "cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_base.h"

/* BEGIN TrajectoryProfileRamp ****************************************************************************************/
/// Class providing a HardwareInterface publishing velocities.
class  TrajectoryProfileRamp: public TrajectoryProfileBase
{
    public:
        TrajectoryProfileRamp(const cob_cartesian_controller::CartesianActionStruct& params)
        : TrajectoryProfileBase(params)
        {        }

        ~TrajectoryProfileRamp() {}
        virtual cob_cartesian_controller::ProfileTimings getProfileTimings(double Se, double te, double accl, double vel, bool calcMaxTe);
        virtual bool generatePath(cob_cartesian_controller::PathArray &pa);
        virtual bool calculateProfile(std::vector<double> *path_matrix,
                                      const double Se_lin, const double Se_rot,
                                      geometry_msgs::Pose start);
        virtual std::vector<double> getTrajectory(double se, double accl, double vel, double t_ipo,
                                                  double steps_tb, double steps_tv, double steps_te, double tb, double tv, double te);

    private:
        cob_cartesian_controller::ProfileTimings pt_max_;
};
/* END TrajectoryProfileRamp **********************************************************************************************/

#endif /* COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_RAMP_H_ */
