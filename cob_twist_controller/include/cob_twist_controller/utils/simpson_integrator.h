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
 *   Author: Felix Messmer, email: felix.messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: November, 2015
 *
 * \brief
 *   This Class provides a helper for integrating velocities using Simpson method
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_UTILS_SIMPSON_INTEGRATOR_H
#define COB_TWIST_CONTROLLER_UTILS_SIMPSON_INTEGRATOR_H

#include <ros/ros.h>
#include <kdl/jntarray.hpp>

#include "cob_twist_controller/utils/moving_average.h"

class SimpsonIntegrator
{
    public:
        explicit SimpsonIntegrator(const uint8_t dof,
                                   const uint16_t ma_size)
        {
            dof_ = dof;
            ma_.assign(dof_, MovingAvg_double_t(ma_size));
            last_update_time_ = ros::Time(0.0);
            last_period_ = ros::Duration(0.0);
        }
        ~SimpsonIntegrator(){}
        
        
        bool updateIntegration(const KDL::JntArray& q_dot_ik,
                               const KDL::JntArray& current_q,
                               std::vector<double>& pos,
                               std::vector<double>& vel)
        {
            now_ = ros::Time::now();
            ros::Duration period = now_ - last_update_time_;

            bool value_valid = false;
            pos.clear();
            vel.clear();

            // ToDo: Test this and find good threshold
            if (period.toSec() > 2*last_period_.toSec())  // missed about a cycle
            {
                ROS_WARN("reset Integration");
                // resetting outdated values
                vel_last_.clear();
                vel_before_last_.clear();
            }

            if (!vel_before_last_.empty())
            {
                for (unsigned int i = 0; i < dof_; ++i)
                {
                    // Simpson
                    double integration_value = static_cast<double>(period.toSec() / 6.0 * (vel_before_last_[i] + 4.0 * (vel_before_last_[i] + vel_last_[i]) + vel_before_last_[i] + vel_last_[i] + q_dot_ik(i)) + current_q(i));
                    ma_[i].addElement(integration_value);
                    double avg = 0.0;
                    ma_[i].calcWeightedMovingAverage(avg);
                    pos.push_back(avg);
                    vel.push_back(q_dot_ik(i));
                }
                value_valid = true;
            }

            // Continuously shift the vectors for simpson integration
            vel_before_last_.clear();
            for (unsigned int i=0; i < vel_last_.size(); ++i)
            {
                vel_before_last_.push_back(vel_last_[i]);
            }

            vel_last_.clear();
            for (unsigned int i=0; i < q_dot_ik.rows(); ++i)
            {
                vel_last_.push_back(q_dot_ik(i));
            }

            last_update_time_ = now_;
            last_period_ = period;
            return value_valid;
        }

    private:
        std::vector<MovingAvg_double_t> ma_;
        uint8_t dof_;
        std::vector<double> vel_last_, vel_before_last_;
        ros::Time now_, last_update_time_;
        ros::Duration last_period_;
};

#endif  // COB_TWIST_CONTROLLER_UTILS_SIMPSON_INTEGRATOR_H
