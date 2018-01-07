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
