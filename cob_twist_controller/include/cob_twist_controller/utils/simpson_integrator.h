/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef COB_TWIST_CONTROLLER_UTILS_SIMPSON_INTEGRATOR_H
#define COB_TWIST_CONTROLLER_UTILS_SIMPSON_INTEGRATOR_H

#include <vector>

#include <ros/ros.h>
#include <kdl/jntarray.hpp>
#include <std_msgs/Float64MultiArray.h>

#include "cob_twist_controller/utils/moving_average.h"

class SimpsonIntegrator
{
    public:
        explicit SimpsonIntegrator(const uint8_t dof, const double integrator_smoothing = 0.2)
            : dof_(dof),
              integrator_smoothing_(integrator_smoothing),
              last_update_time_(ros::Time(0.0))
        {
            for (uint8_t i = 0; i < dof_; i++)
            {
                ma_vel_.push_back(new MovingAvgExponential_double_t(integrator_smoothing_));
                ma_pos_.push_back(new MovingAvgExponential_double_t(integrator_smoothing_));
            }

            /// nh_ = ros::NodeHandle("simpson_debug");
            /// q_dot_ik_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("q_dot_ik", 1);
            /// q_dot_avg_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("q_dot_avg", 1);
            /// q_simpson_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("q_simpson", 1);
            /// q_simpson_avg_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("q_simpson_avg", 1);
        }

        ~SimpsonIntegrator()
        {}

        void resetIntegration()
        {
            // resetting outdated values
            vel_last_.clear();
            vel_before_last_.clear();

            // resetting moving average
            for (unsigned int i = 0; i < dof_; ++i)
            {
                ma_vel_[i]->reset();
                ma_pos_[i]->reset();
            }
        }

        bool updateIntegration(const KDL::JntArray& q_dot_ik,
                               const KDL::JntArray& current_q,
                               std::vector<double>& pos,
                               std::vector<double>& vel)
        {
            ros::Time now = ros::Time::now();
            ros::Duration period = now - last_update_time_;
            last_update_time_ = now;

            bool value_valid = false;
            pos.clear();
            vel.clear();

            // ToDo: Test these conditions and find good thresholds
            if (period.toSec() > ros::Duration(0.5).toSec())  // missed about 'max_command_silence'
            {
                ROS_WARN_STREAM("reset Integration: " << period.toSec());
                resetIntegration();
            }

            // smooth incoming velocities
            KDL::JntArray q_dot_avg(dof_);
            /// std_msgs::Float64MultiArray q_dot_ik_msg;
            /// std_msgs::Float64MultiArray q_dot_avg_msg;
            for (unsigned int i = 0; i < dof_; ++i)
            {
                ma_vel_[i]->addElement(q_dot_ik(i));
                double avg_vel = 0.0;
                if (ma_vel_[i]->calcMovingAverage(avg_vel))
                {
                    q_dot_avg(i) = avg_vel;
                }
                else
                {
                    q_dot_avg(i) = q_dot_ik(i);
                }

                /// q_dot_ik_msg.data.push_back(q_dot_ik(i));
                /// q_dot_avg_msg.data.push_back(q_dot_avg(i));
            }

            /// std_msgs::Float64MultiArray q_simpson_msg;
            /// std_msgs::Float64MultiArray q_simpson_avg_msg;
            if (!vel_before_last_.empty())
            {
                for (unsigned int i = 0; i < dof_; ++i)
                {
                    // Simpson
                    double integration_value = static_cast<double>(period.toSec() / 6.0 * (vel_before_last_[i] + 4.0 * (vel_before_last_[i] + vel_last_[i]) + vel_before_last_[i] + vel_last_[i] + q_dot_avg(i)) + current_q(i));

                    // smooth outgoing positions
                    ma_pos_[i]->addElement(integration_value);
                    double avg_pos = 0.0;
                    if (!ma_pos_[i]->calcMovingAverage(avg_pos))
                    {
                        ROS_ERROR("calcMovingAverage failed");
                        avg_pos = integration_value;
                    }

                    pos.push_back(avg_pos);
                    vel.push_back(q_dot_avg(i));

                    /// q_simpson_msg.data.push_back(integration_value);
                    /// q_simpson_avg_msg.data.push_back(avg_pos);
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
            for (unsigned int i=0; i < q_dot_avg.rows(); ++i)
            {
                vel_last_.push_back(q_dot_avg(i));
            }

            /// q_dot_ik_pub_.publish(q_dot_ik_msg);
            /// q_dot_avg_pub_.publish(q_dot_avg_msg);
            /// q_simpson_pub_.publish(q_simpson_msg);
            /// q_simpson_avg_pub_.publish(q_simpson_avg_msg);

            return value_valid;
        }

    private:
        std::vector<MovingAvgBase_double_t*> ma_vel_;
        std::vector<MovingAvgBase_double_t*> ma_pos_;
        uint8_t dof_;
        double integrator_smoothing_;
        std::vector<double> vel_last_, vel_before_last_;
        ros::Time last_update_time_;

        // debug
        /// ros::NodeHandle nh_;
        /// ros::Publisher q_dot_ik_pub_;
        /// ros::Publisher q_dot_avg_pub_;
        /// ros::Publisher q_simpson_pub_;
        /// ros::Publisher q_simpson_avg_pub_;
};

#endif  // COB_TWIST_CONTROLLER_UTILS_SIMPSON_INTEGRATOR_H
