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
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This header contains the interface description of all available
 *   controller interfaces (Velocity/Position/Trajectory/JointStates).
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_CONTROLLER_INTERFACES_CONTROLLER_INTERFACE_BASE_H
#define COB_TWIST_CONTROLLER_CONTROLLER_INTERFACES_CONTROLLER_INTERFACE_BASE_H

#include <vector>
#include "ros/ros.h"

#include "cob_twist_controller/cob_twist_controller_data_types.h"

/// Base class for controller interfaces.
class ControllerInterfaceBase
{
    public:
        explicit ControllerInterfaceBase(ros::NodeHandle& nh,
                                         const TwistControllerParams& params):
            nh_(nh),
            params_(params)
        {}

        virtual ~ControllerInterfaceBase() {}

        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q) = 0;

    protected:
        const TwistControllerParams& params_;
        ros::NodeHandle& nh_;
        ros::Publisher pub_;
};

/// Base class for controller interfaces using position integration
class ControllerInterfacePositionBase : public ControllerInterfaceBase
{
    public:
        explicit ControllerInterfacePositionBase(ros::NodeHandle& nh,
                                                 const TwistControllerParams& params,
                                                 const uint16_t ma_size)
        : ControllerInterfaceBase(nh, params)
        {
            ma_.assign(params.dof, MovingAvg_double_t(ma_size));
            last_update_time_ = ros::Time(0.0);
            last_period_ = ros::Duration(0.0);
        }

        ~ControllerInterfacePositionBase() {}

        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q) = 0;

        bool updateIntegration(const KDL::JntArray& q_dot_ik,
                               const KDL::JntArray& current_q)
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
                for (unsigned int i = 0; i < params_.dof; ++i)
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

    protected:
        std::vector<MovingAvg_double_t> ma_;
        std::vector<double> vel_last_, vel_before_last_;
        ros::Time now_, last_update_time_;
        ros::Duration last_period_;
        std::vector<double> pos, vel;
};

#endif  // COB_TWIST_CONTROLLER_CONTROLLER_INTERFACES_CONTROLLER_INTERFACE_BASE_H
