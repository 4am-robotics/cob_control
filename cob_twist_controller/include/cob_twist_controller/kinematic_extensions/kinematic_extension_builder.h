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


#ifndef COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_BUILDER_H
#define COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_BUILDER_H

#include <ros/ros.h>
#include <Eigen/Geometry>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/kinematic_extensions/kinematic_extension_base.h"
#include "cob_twist_controller/kinematic_extensions/kinematic_extension_dof.h"
#include "cob_twist_controller/kinematic_extensions/kinematic_extension_urdf.h"
#include "cob_twist_controller/kinematic_extensions/kinematic_extension_lookat.h"


/* BEGIN KinematicExtensionBuilder *****************************************************************************************/
/// Class providing a static method to create kinematic extension objects.
class KinematicExtensionBuilder
{
    public:
        KinematicExtensionBuilder() {}
        ~KinematicExtensionBuilder() {}

        static KinematicExtensionBase* createKinematicExtension(const TwistControllerParams& params);
};
/* END KinematicExtensionBuilder *******************************************************************************************/


/* BEGIN KinematicExtensionNone ****************************************************************************************/
/// Class implementing the interface in case KinematicExtension is disabled.
class KinematicExtensionNone : public KinematicExtensionBase
{
    public:
        explicit KinematicExtensionNone(const TwistControllerParams& params)
        : KinematicExtensionBase(params)
        {}

        ~KinematicExtensionNone() {}

        bool initExtension();
        KDL::Jacobian adjustJacobian(const KDL::Jacobian& jac_chain);
        JointStates adjustJointStates(const JointStates& joint_states);
        LimiterParams adjustLimiterParams(const LimiterParams& limiter_params);
        void processResultExtension(const KDL::JntArray& q_dot_ik);
};
/* END KinematicExtensionNone **********************************************************************************************/

#endif  // COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_BUILDER_H
