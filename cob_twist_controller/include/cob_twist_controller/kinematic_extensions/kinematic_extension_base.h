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


#ifndef COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_BASE_H
#define COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_BASE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "cob_twist_controller/cob_twist_controller_data_types.h"


/// Base class for kinematic extensions.
class KinematicExtensionBase
{
    public:
        explicit KinematicExtensionBase(const TwistControllerParams& params):
            params_(params)
        {
            /// give tf_listener_ some time to fill buffer
            ros::Duration(0.5).sleep();
        }

        virtual ~KinematicExtensionBase() {}

        virtual bool initExtension() = 0;
        virtual KDL::Jacobian adjustJacobian(const KDL::Jacobian& jac_chain) = 0;
        virtual JointStates adjustJointStates(const JointStates& joint_states) = 0;
        virtual LimiterParams adjustLimiterParams(const LimiterParams& limiter_params) = 0;
        virtual void processResultExtension(const KDL::JntArray& q_dot_ik) = 0;

    protected:
        ros::NodeHandle nh_;
        tf::TransformListener tf_listener_;
        const TwistControllerParams& params_;
};

#endif  // COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_BASE_H
