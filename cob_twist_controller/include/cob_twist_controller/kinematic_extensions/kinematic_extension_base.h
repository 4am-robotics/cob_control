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
