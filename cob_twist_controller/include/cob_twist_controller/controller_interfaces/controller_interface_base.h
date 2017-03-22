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
#include "cob_twist_controller/utils/simpson_integrator.h"

namespace cob_twist_controller
{

/// Base class for controller interfaces.
class ControllerInterfaceBase
{
    public:
        virtual ~ControllerInterfaceBase() {}

        virtual void initialize(ros::NodeHandle& nh,
                                const TwistControllerParams& params) = 0;
        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q) = 0;

    protected:
        ControllerInterfaceBase() {}

        TwistControllerParams params_;
        ros::NodeHandle nh_;
        ros::Publisher pub_;
};

/// Base class for controller interfaces using position integration
class ControllerInterfacePositionBase : public ControllerInterfaceBase
{
    public:
        ~ControllerInterfacePositionBase() {}

        bool updateIntegration(const KDL::JntArray& q_dot_ik,
                               const KDL::JntArray& current_q)
        {
            ros::Time now = ros::Time::now();
            period_ = now - last_update_time_;
            last_update_time_ = now;
            return integrator_->updateIntegration(q_dot_ik, current_q, pos_, vel_);
        }

    protected:
        ControllerInterfacePositionBase() {}

        boost::shared_ptr<SimpsonIntegrator> integrator_;
        std::vector<double> pos_, vel_;
        ros::Time last_update_time_;
        ros::Duration period_;
};

}

#endif  // COB_TWIST_CONTROLLER_CONTROLLER_INTERFACES_CONTROLLER_INTERFACE_BASE_H
