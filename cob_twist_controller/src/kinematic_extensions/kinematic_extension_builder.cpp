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
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This header contains the interface description for extening the
 *   kinematic chain with additional degrees of freedom, e.g. base_active or lookat
 *
 ****************************************************************/

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
