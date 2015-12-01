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

#ifndef COB_CARTESIAN_CONTROLLER_DATA_STRUCTURES_H_
#define COB_CARTESIAN_CONTROLLER_DATA_STRUCTURES_H_

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <exception>

namespace cob_cartesian_controller
{
    struct ProfileStruct
    {
        double t_ipo;
        unsigned int profile_type;
        double vel, accl;
        double Se_max;
    };

    struct ProfileTimings
    {
        double tb, te, tv;
        unsigned int steps_tb, steps_te, steps_tv;
        bool ok;
        double vel;
    };

    struct MoveLinStruct
    {
        geometry_msgs::Pose start, end;
    };

    struct MoveCircStruct
    {
        geometry_msgs::Pose pose_center;
        double start_angle, end_angle;
        double radius;
        bool rotate_only;
    };

    struct CartesianActionStruct
    {
        unsigned int move_type;
        MoveLinStruct move_lin;
        MoveCircStruct move_circ;
        ProfileStruct profile;
    };



    class PathArray{
        public:
            PathArray(double Se, std::vector<double> array):
                Se_(Se),
                array_(array)
            {
                calcTe_ = false;
            }

            ~PathArray()
            {
                array_.clear();
            }

            bool calcTe_;
            double Se_;
            std::vector<double> array_;

    };

    class PathMatrix{
            public:
                PathMatrix(PathArray &pa1,
                           PathArray &pa2)
                {
                    pm_.push_back(pa1);
                    pm_.push_back(pa2);

                }
                ~PathMatrix()
                {
                    pm_.clear();
                }

                double getMaxSe();
                std::vector<PathArray> pm_;
        };

    inline double PathMatrix::getMaxSe()
    {
        double se_max = 0;

        for(int i=0; i<pm_.size(); i++)
        {
            if(se_max < fabs(pm_[i].Se_))
            {
                se_max = fabs(pm_[i].Se_);
            }
        }
        return se_max;
    }
}//namespace
#endif /* COB_CARTESIAN_CONTROLLER_DATA_STRUCTURES_H_ */
