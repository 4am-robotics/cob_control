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
    };

    struct MoveLinStruct
    {
        geometry_msgs::Pose start, end;
        bool rotate_only;
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
            PathArray(double idx, double Se, double start_value, std::vector<double> array):
                Se_(Se),
                array_(array),
                start_value_(start_value)
            {
                idx_= idx;
                calcTe_=false;
            }

            ~PathArray()
            {
                array_.clear();
            }

            unsigned int getIdx();
            bool getCalcTe();
            double getSe();
            std::vector<double> getArray();
            void setCalcTe(bool calcTe);
            void setIdx(unsigned int idx);
            void setArray(std::vector<double> array);
            double getStartValue();

        private:
            unsigned int idx_;
            bool calcTe_;
            double Se_;
            std::vector<double> array_;
            double start_value_;

    };

    inline double PathArray::getStartValue()
    {
        return start_value_;
    }

    inline void PathArray::setArray(std::vector<double> array)
    {
        array_ = array;
    }

    inline void PathArray::setCalcTe(bool calcTe)
    {
        calcTe_ = calcTe;
    }

    inline void PathArray::setIdx(unsigned int idx)
    {
        idx_=idx;
    }

    inline unsigned int PathArray::getIdx()
    {
        return idx_;
    }

    inline bool PathArray::getCalcTe()
    {
        return calcTe_;
    }

    inline double PathArray::getSe()
    {
        return Se_;
    }

    inline std::vector<double> PathArray::getArray()
    {
        return array_;
    }


    class PathMatrix{
            public:
                PathMatrix(PathArray &pa1,
                           PathArray &pa2,
                           PathArray &pa3,
                           PathArray &pa4)
//                           PathArray &pa5,
//                           PathArray &pa6)
                {
                    pm_.push_back(pa1);
                    pm_.push_back(pa2);
                    pm_.push_back(pa3);
                    pm_.push_back(pa4);
//                    pm_.push_back(pa5);
//                    pm_.push_back(pa6);
                }
                ~PathMatrix()
                {
                    pm_.clear();
                }

                std::vector<PathArray> getSortedMatrix();

            private:
                void sortMatrixRows();
                std::vector<PathArray> pm_;

        };

    inline std::vector<PathArray> PathMatrix::getSortedMatrix()
    {
        sortMatrixRows();
        return pm_;
    }

    inline void PathMatrix::sortMatrixRows()
    {
        std::vector<double> tmp;
        PathArray temp(0, 0, 0.0, tmp);
        for(int j = 0; j<pm_.size(); j++)
        {
            for(int i = 0; i<pm_.size()-1; i++)
            {
                if(std::fabs(pm_[i].getSe()) < std::fabs(pm_[i+1].getSe()))
                {
                    temp = pm_[i];
                    pm_[i] = pm_[i+1];
                    pm_[i+1] = temp;
                }
            }
        }
        pm_[0].setCalcTe(true); // Reference Time for profile interpolation (Te)
    }


}//namespace
#endif /* COB_CARTESIAN_CONTROLLER_DATA_STRUCTURES_H_ */
