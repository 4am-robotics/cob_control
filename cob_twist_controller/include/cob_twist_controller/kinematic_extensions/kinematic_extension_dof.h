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


#ifndef COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_DOF_H
#define COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_DOF_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Geometry>

#include "cob_twist_controller/kinematic_extensions/kinematic_extension_base.h"

/* BEGIN KinematicExtensionDOF ****************************************************************************************/
/// Abstract Helper Class to be used for Cartesian KinematicExtensions based on enabled DoFs.
class KinematicExtensionDOF : public KinematicExtensionBase
{
    public:
        explicit KinematicExtensionDOF(const TwistControllerParams& params)
        : KinematicExtensionBase(params)
        {}

        ~KinematicExtensionDOF() {}

        virtual bool initExtension() = 0;
        virtual KDL::Jacobian adjustJacobian(const KDL::Jacobian& jac_chain) = 0;
        virtual JointStates adjustJointStates(const JointStates& joint_states) = 0;
        virtual LimiterParams adjustLimiterParams(const LimiterParams& limiter_params) = 0;
        virtual void processResultExtension(const KDL::JntArray& q_dot_ik) = 0;

        KDL::Jacobian adjustJacobianDof(const KDL::Jacobian& jac_chain, const KDL::Frame eb_frame_ct, const KDL::Frame cb_frame_eb, const ActiveCartesianDimension active_dim);

    protected:
        unsigned int ext_dof_;
        std::vector<std::string> joint_names_;
        JointStates joint_states_;
        std::vector<double> limits_max_;
        std::vector<double> limits_min_;
        std::vector<double> limits_vel_;
        std::vector<double> limits_acc_;
};
/* END KinematicExtensionDOF **********************************************************************************************/

/* BEGIN KinematicExtensionBaseActive ****************************************************************************************/
/// Class implementing a mobile base KinematicExtension with Cartesian DoFs (lin_x, lin_y, rot_z) enabled (i.e. 2D).
class KinematicExtensionBaseActive : public KinematicExtensionDOF
{
    public:
        explicit KinematicExtensionBaseActive(const TwistControllerParams& params)
        : KinematicExtensionDOF(params)
        {}

        ~KinematicExtensionBaseActive() {}

        bool initExtension();
        KDL::Jacobian adjustJacobian(const KDL::Jacobian& jac_chain);
        JointStates adjustJointStates(const JointStates& joint_states);
        LimiterParams adjustLimiterParams(const LimiterParams& limiter_params);
        void processResultExtension(const KDL::JntArray& q_dot_ik);

        void baseTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);

    private:
        ros::Publisher base_vel_pub_;

        double min_vel_lin_base_;
        double min_vel_rot_base_;
        double max_vel_lin_base_;
        double max_vel_rot_base_;
};
/* END KinematicExtensionBaseActive **********************************************************************************************/

#endif  // COB_TWIST_CONTROLLER_KINEMATIC_EXTENSIONS_KINEMATIC_EXTENSION_DOF_H
