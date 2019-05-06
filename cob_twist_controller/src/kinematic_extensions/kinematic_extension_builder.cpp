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


#include <eigen_conversions/eigen_kdl.h>
#include "cob_twist_controller/kinematic_extensions/kinematic_extension_builder.h"


/* BEGIN KinematicExtensionBuilder *****************************************************************************************/
/**
 * Static builder method to create kinematic extensions based on given parameterization.
 */
KinematicExtensionBase* KinematicExtensionBuilder::createKinematicExtension(const TwistControllerParams& params)
{
    KinematicExtensionBase* keb = NULL;

    switch (params.kinematic_extension)
    {
        case NO_EXTENSION:
            keb = new KinematicExtensionNone(params);
            break;
        case BASE_COMPENSATION:
            // nothing to do here for BASE_COMPENSATION - only affects twist subscription callback
            keb = new KinematicExtensionNone(params);
            break;
        case BASE_ACTIVE:
            keb = new KinematicExtensionBaseActive(params);
            break;
        case COB_TORSO:
            keb = new KinematicExtensionTorso(params);
            break;
        case LOOKAT:
            keb = new KinematicExtensionLookat(params);
            break;
        default:
            ROS_ERROR("KinematicExtension %d not defined! Using default: 'NO_EXTENSION'!", params.kinematic_extension);
            keb = new KinematicExtensionNone(params);
            break;
    }
    if (!keb->initExtension())
    {
        ROS_ERROR("Failed to createKinematicExtension %d! Using default: 'NO_EXTENSION'!", params.kinematic_extension);
        keb = NULL;
    }

    return keb;
}
/* END KinematicExtensionBuilder *******************************************************************************************/


/* BEGIN KinematicExtensionNone ********************************************************************************************/

bool KinematicExtensionNone::initExtension()
{
    return true;
}

/**
 * Method adjusting the Jacobian used in inverse differential computation. No changes applied.
 */
KDL::Jacobian KinematicExtensionNone::adjustJacobian(const KDL::Jacobian& jac_chain)
{
    return jac_chain;
}

/**
 * Method adjusting the JointStates used in inverse differential computation and limiters. No changes applied.
 */
JointStates KinematicExtensionNone::adjustJointStates(const JointStates& joint_states)
{
    return joint_states;
}

/**
 * Method adjusting the LimiterParams used in limiters. No changes applied.
 */
LimiterParams KinematicExtensionNone::adjustLimiterParams(const LimiterParams& limiter_params)
{
    return limiter_params;
}

/**
 * Method processing the partial result realted to the kinematic extension. Nothing to be done.
 */
void KinematicExtensionNone::processResultExtension(const KDL::JntArray& q_dot_ik)
{
    return;
}
/* END KinematicExtensionNone **********************************************************************************************/
