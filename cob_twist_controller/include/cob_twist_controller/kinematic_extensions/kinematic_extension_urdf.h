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

#ifndef COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_URDF_H
#define COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_URDF_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Geometry>

#include "cob_twist_controller/kinematic_extensions/kinematic_extension_base.h"

/* BEGIN KinematicExtensionURDF ****************************************************************************************/
/// Abstract Helper Class to be used for Cartesian KinematicExtensions based on URDF.
class KinematicExtensionURDF : public KinematicExtensionBase
{
    public:
        explicit KinematicExtensionURDF(const TwistControllerParams& params)
        : KinematicExtensionBase(params)
        {}

        ~KinematicExtensionURDF() {}

        bool initExtension();
        virtual KDL::Jacobian adjustJacobian(const KDL::Jacobian& jac_chain);
        virtual JointStates adjustJointStates(const JointStates& joint_states);
        virtual LimiterParams adjustLimiterParams(const LimiterParams& limiter_params);
        virtual void processResultExtension(const KDL::JntArray& q_dot_ik);

        void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    protected:
        ros::Publisher command_pub_;
        ros::Subscriber joint_state_sub_;

        std::string ext_base_;
        std::string ext_tip_;
        KDL::Chain chain_;
        unsigned int ext_dof_;
        std::vector<std::string> joint_names_;
        JointStates joint_states_;
        std::vector<double> limits_max_;
        std::vector<double> limits_min_;
        std::vector<double> limits_vel_;
        std::vector<double> limits_acc_;
};
/* END KinematicExtensionURDF **********************************************************************************************/


/* BEGIN KinematicExtensionTorso ****************************************************************************************/
/// Class implementing a KinematicExtension for Torso based on URDF.
class KinematicExtensionTorso : public KinematicExtensionURDF
{
    public:
        explicit KinematicExtensionTorso(const TwistControllerParams& params)
        : KinematicExtensionURDF(params)
        {
            ext_base_ = "torso_base_link";
            ext_tip_ = params.chain_base_link;

            if (!initExtension())
            {
                ROS_ERROR("Initialization failed");
            }

            joint_state_sub_ = nh_.subscribe("/torso/joint_states", 1, &KinematicExtensionURDF::jointstateCallback, dynamic_cast<KinematicExtensionURDF*>(this));
            command_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/torso/joint_group_velocity_controller/command", 1);
        }

        ~KinematicExtensionTorso() {}
};
/* END KinematicExtensionTorso **********************************************************************************************/

#endif  // COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_URDF_H
