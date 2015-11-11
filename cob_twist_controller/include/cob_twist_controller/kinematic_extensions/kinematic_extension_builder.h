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
 *   This header contains the interface description for extending the
 *   kinematic chain with additional degrees of freedom, e.g. base_active or lookat
 *
 ****************************************************************/

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
