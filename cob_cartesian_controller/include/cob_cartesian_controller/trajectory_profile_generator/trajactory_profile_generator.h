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
#ifndef TRAJECTORY_PROFILE_BUILDER_H_
#define TRAJECTORY_PROFILE_BUILDER_H_

#include "cob_cartesian_controller/trajectory_profile_generator/trajactory_profile_generator_base.h"
#include "cob_cartesian_controller/cartesian_controller_data_types.h"


/* BEGIN TrajectoryProfileBuilder *****************************************************************************************/
class TrajectoryProfileBuilder
{
    public:
        TrajectoryProfileBuilder() {}
        ~TrajectoryProfileBuilder() {}

        static TrajectoryProfileBase* createProfile(const cob_cartesian_controller::CartesianActionStruct& params);
};
/* END TrajectoryGeneratorBuilder *******************************************************************************************/


/* BEGIN TrajectoryProfileRamp ****************************************************************************************/
/// Class providing a HardwareInterface publishing velocities.
class  TrajectoryProfileRamp: public TrajectoryProfileBase
{
    public:
        TrajectoryProfileRamp(const cob_cartesian_controller::CartesianActionStruct& params)
        : TrajectoryProfileBase(params)
        {        }

        ~TrajectoryProfileRamp() {}

        virtual void getProfileTimings(double Se_max, double accl, double vel);
        virtual void generatePath(cob_cartesian_controller::PathArray &pa);
        virtual bool calculateProfile(std::vector<double>* path_matrix,
                                      double Se, double Se_roll, double Se_pitch, double Se_yaw,
                                      geometry_msgs::Pose start);



    protected:
        cob_cartesian_controller::ProfileTimings pt_;
};
/* END TrajectoryProfileRamp **********************************************************************************************/


/* BEGIN TrajectoryProfileSinoid ****************************************************************************************/
/// Class providing a HardwareInterface publishing positions.
class TrajectoryProfileSinoid : public TrajectoryProfileBase
{
    public:
        TrajectoryProfileSinoid(const cob_cartesian_controller::CartesianActionStruct& params)
        : TrajectoryProfileBase(params)
        {        }

        ~TrajectoryProfileSinoid() {}

        virtual void getProfileTimings(double Se_max, double accl, double vel);
        virtual void generatePath(cob_cartesian_controller::PathArray &pa);
        virtual bool calculateProfile(std::vector<double>* path_matrix,
                                      double Se, double Se_roll, double Se_pitch, double Se_yaw,
                                      geometry_msgs::Pose start);

    protected:
        cob_cartesian_controller::ProfileTimings pt_;
};
/* END TrajectoryProfileSinoid **********************************************************************************************/

#endif
