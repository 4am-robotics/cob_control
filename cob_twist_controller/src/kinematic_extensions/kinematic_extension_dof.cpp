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


#include <limits>
#include <eigen_conversions/eigen_kdl.h>
#include "cob_twist_controller/kinematic_extensions/kinematic_extension_dof.h"

/* BEGIN KinematicExtensionDOF ********************************************************************************************/
/**
 * Helper function adjusting the Jacobian used in inverse differential computation based on the Cartesian DoFs enabled in 'adjustJacobian()'.
 * @param jac_chain The jacobian of the primary kinematic chain.
 * @param eb_frame_ct The transformation from base_frame of the extension (eb) to the tip_frame of the primary chain (ct).
 * @param cb_frame_eb The transformation from base_frame of the primary chain (cb) to the base_frame of the extension (eb).
 * @param active_dim The binary vector of active dimensions.
 * @return The extended Jacobian
 */
KDL::Jacobian KinematicExtensionDOF::adjustJacobianDof(const KDL::Jacobian& jac_chain, const KDL::Frame eb_frame_ct, const KDL::Frame cb_frame_eb, const ActiveCartesianDimension active_dim)
{
    /// compose jac_full considering kinematical extension
    KDL::Jacobian jac_full;

    // jacobian matrix for the extension
    Eigen::Matrix<double, 6, Eigen::Dynamic> jac_ext;
    jac_ext.resize(6, ext_dof_);
    jac_ext.setZero();

    // rotation from base_frame of primary chain to base_frame of extension (eb)
    Eigen::Quaterniond quat_cb;
    tf::quaternionKDLToEigen(cb_frame_eb.M, quat_cb);
    Eigen::Matrix3d rot_cb = quat_cb.toRotationMatrix();

    /// angular velocities
    // omega wrt eb
    Eigen::Vector3d w_x_eb(1, 0, 0);
    Eigen::Vector3d w_y_eb(0, 1, 0);
    Eigen::Vector3d w_z_eb(0, 0, 1);

    // transform to cb
    Eigen::Vector3d w_x_cb = quat_cb * w_x_eb;
    Eigen::Vector3d w_y_cb = quat_cb * w_y_eb;
    Eigen::Vector3d w_z_cb = quat_cb * w_z_eb;

    /// linear velocities
    // vector from base_frame of extension (eb) to endeffector (ct)
    Eigen::Vector3d p_eb(eb_frame_ct.p.x(), eb_frame_ct.p.y(), eb_frame_ct.p.z());

    // transform to cb
    Eigen::Vector3d p_cb = quat_cb * p_eb;

    // Calculate tangential velocity
    Eigen::Vector3d vel_x_cb = w_x_cb.cross(p_cb);
    Eigen::Vector3d vel_y_cb = w_y_cb.cross(p_cb);
    Eigen::Vector3d vel_z_cb = w_z_cb.cross(p_cb);

    /// Fill Jacobian column by column
    // effect of lin_x motion
    jac_ext(0, 0) = rot_cb(0, 0) * active_dim.lin_x;
    jac_ext(1, 0) = rot_cb(1, 0) * active_dim.lin_x;
    jac_ext(2, 0) = rot_cb(2, 0) * active_dim.lin_x;
    jac_ext(3, 0) = 0.0;
    jac_ext(4, 0) = 0.0;
    jac_ext(5, 0) = 0.0;

    // effect of lin_y motion
    jac_ext(0, 1) = rot_cb(0, 1) * active_dim.lin_y;
    jac_ext(1, 1) = rot_cb(1, 1) * active_dim.lin_y;
    jac_ext(2, 1) = rot_cb(2, 1) * active_dim.lin_y;
    jac_ext(3, 1) = 0.0;
    jac_ext(4, 1) = 0.0;
    jac_ext(5, 1) = 0.0;

    // effect of lin_z motion
    jac_ext(0, 2) = rot_cb(0, 2) * active_dim.lin_z;
    jac_ext(1, 2) = rot_cb(1, 2) * active_dim.lin_z;
    jac_ext(2, 2) = rot_cb(2, 2) * active_dim.lin_z;
    jac_ext(3, 2) = 0.0;
    jac_ext(4, 2) = 0.0;
    jac_ext(5, 2) = 0.0;

    // effect of rot_x motion
    jac_ext(0, 3) = vel_x_cb(0) * active_dim.rot_x;
    jac_ext(1, 3) = vel_x_cb(1) * active_dim.rot_x;
    jac_ext(2, 3) = vel_x_cb(2) * active_dim.rot_x;
    jac_ext(3, 3) = w_x_cb(0) * active_dim.rot_x;
    jac_ext(4, 3) = w_x_cb(1) * active_dim.rot_x;
    jac_ext(5, 3) = w_x_cb(2) * active_dim.rot_x;

    // effect of rot_y motion
    jac_ext(0, 4) = vel_y_cb(0) * active_dim.rot_y;
    jac_ext(1, 4) = vel_y_cb(1) * active_dim.rot_y;
    jac_ext(2, 4) = vel_y_cb(2) * active_dim.rot_y;
    jac_ext(3, 4) = w_y_cb(0) * active_dim.rot_y;
    jac_ext(4, 4) = w_y_cb(1) * active_dim.rot_y;
    jac_ext(5, 4) = w_y_cb(2) * active_dim.rot_y;

    // effect of rot_z motion
    jac_ext(0, 5) = vel_z_cb(0) * active_dim.rot_z;
    jac_ext(1, 5) = vel_z_cb(1) * active_dim.rot_z;
    jac_ext(2, 5) = vel_z_cb(2) * active_dim.rot_z;
    jac_ext(3, 5) = w_z_cb(0) * active_dim.rot_z;
    jac_ext(4, 5) = w_z_cb(1) * active_dim.rot_z;
    jac_ext(5, 5) = w_z_cb(2) * active_dim.rot_z;

    // scale with extension_ratio
    jac_ext *= params_.extension_ratio;

    // combine Jacobian of primary chain and extension
    Matrix6Xd_t jac_full_matrix;
    jac_full_matrix.resize(6, jac_chain.data.cols() + jac_ext.cols());
    jac_full_matrix << jac_chain.data, jac_ext;
    jac_full.resize(jac_chain.data.cols() + jac_ext.cols());
    jac_full.data << jac_full_matrix;

    return jac_full;
}
/* END KinematicExtensionDOF **********************************************************************************************/


/* BEGIN KinematicExtensionBaseActive ********************************************************************************************/
bool KinematicExtensionBaseActive::initExtension()
{
    base_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base/command", 1);

    min_vel_lin_base_ = 0.005;  // used to avoid infinitesimal motion
    min_vel_rot_base_ = 0.005;  // used to avoid infinitesimal motion
    max_vel_lin_base_ = 0.5;
    max_vel_rot_base_ = 0.5;

    this->ext_dof_ = 6;
    this->joint_states_.last_q_.resize(ext_dof_);
    this->joint_states_.last_q_dot_.resize(ext_dof_);
    this->joint_states_.current_q_.resize(ext_dof_);
    this->joint_states_.current_q_dot_.resize(ext_dof_);

    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        limits_max_.push_back(std::numeric_limits<double>::max());
        limits_min_.push_back(-std::numeric_limits<double>::max());
        if (i < 3)
        {
            limits_vel_.push_back(max_vel_lin_base_);
        }
        else
        {
            limits_vel_.push_back(max_vel_rot_base_);
        }
        limits_acc_.push_back(std::numeric_limits<double>::max());
    }

    return true;
}

/**
 * Method adjusting the Jacobian used in inverse differential computation. Enable Cartesian DoFs (lin_x, lin_y, rot_z) considering current transformation to main kinematic chain.
 */
KDL::Jacobian KinematicExtensionBaseActive::adjustJacobian(const KDL::Jacobian& jac_chain)
{
    tf::StampedTransform bl_transform_ct, cb_transform_bl;
    KDL::Frame bl_frame_ct, cb_frame_bl;
    ActiveCartesianDimension active_dim;

    /// get required transformations
    try
    {
        ros::Time now = ros::Time(0);
        tf_listener_.waitForTransform("base_link", params_.chain_tip_link, now, ros::Duration(0.5));
        tf_listener_.lookupTransform("base_link", params_.chain_tip_link,  now, bl_transform_ct);

        tf_listener_.waitForTransform(params_.chain_base_link, "base_link", now, ros::Duration(0.5));
        tf_listener_.lookupTransform(params_.chain_base_link, "base_link", now, cb_transform_bl);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    bl_frame_ct.p = KDL::Vector(bl_transform_ct.getOrigin().x(), bl_transform_ct.getOrigin().y(), bl_transform_ct.getOrigin().z());
    bl_frame_ct.M = KDL::Rotation::Quaternion(bl_transform_ct.getRotation().x(), bl_transform_ct.getRotation().y(), bl_transform_ct.getRotation().z(), bl_transform_ct.getRotation().w());

    cb_frame_bl.p = KDL::Vector(cb_transform_bl.getOrigin().x(), cb_transform_bl.getOrigin().y(), cb_transform_bl.getOrigin().z());
    cb_frame_bl.M = KDL::Rotation::Quaternion(cb_transform_bl.getRotation().x(), cb_transform_bl.getRotation().y(), cb_transform_bl.getRotation().z(), cb_transform_bl.getRotation().w());

    /// active base can move in lin_x, lin_y and rot_z
    active_dim.lin_x = 1;
    active_dim.lin_y = 1;
    active_dim.lin_z = 0;

    active_dim.rot_x = 0;
    active_dim.rot_y = 0;
    active_dim.rot_z = 1;

    return adjustJacobianDof(jac_chain, bl_frame_ct, cb_frame_bl, active_dim);
}

/**
 * Method adjusting the JointStates used in inverse differential computation and limiters. Fill neutrally.
 */
JointStates KinematicExtensionBaseActive::adjustJointStates(const JointStates& joint_states)
{
    JointStates js;
    unsigned int chain_dof = joint_states.current_q_.rows();
    js.current_q_.resize(chain_dof + ext_dof_);
    js.last_q_.resize(chain_dof + ext_dof_);
    js.current_q_dot_.resize(chain_dof + ext_dof_);
    js.last_q_dot_.resize(chain_dof + ext_dof_);

    for (unsigned int i = 0; i< chain_dof; i++)
    {
        js.current_q_(i) = joint_states.current_q_(i);
        js.last_q_(i) = joint_states.last_q_(i);
        js.current_q_dot_(i) = joint_states.current_q_dot_(i);
        js.last_q_dot_(i) = joint_states.last_q_dot_(i);
    }
    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        js.current_q_(chain_dof + i) = 0.0;
        js.last_q_(chain_dof + i) = 0.0;
        js.current_q_dot_(chain_dof + i) = 0.0;
        js.last_q_dot_(chain_dof + i) = 0.0;
    }
    return js;
}

/**
 * Method adjusting the LimiterParams used in limiters. Appends limits for BaseActive
 */
LimiterParams KinematicExtensionBaseActive::adjustLimiterParams(const LimiterParams& limiter_params)
{
    LimiterParams lp = limiter_params;
    for (unsigned int i = 0; i < ext_dof_; i++)
    {
        lp.limits_max.push_back(std::numeric_limits<double>::max());
        lp.limits_min.push_back(-std::numeric_limits<double>::max());
        if (i < 3)
        {
            lp.limits_vel.push_back(max_vel_lin_base_);
        }
        else
        {
            lp.limits_vel.push_back(max_vel_rot_base_);
        }
        lp.limits_acc.push_back(std::numeric_limits<double>::max());
    }
    return lp;
}

/**
 * Method processing the partial result related to the kinematic extension. Publish desired Twist to the 'command' topic of the base.
 */
void KinematicExtensionBaseActive::processResultExtension(const KDL::JntArray& q_dot_ik)
{
    geometry_msgs::Twist base_vel_msg;

    base_vel_msg.linear.x = (std::fabs(q_dot_ik(params_.dof)) < min_vel_lin_base_) ? 0.0 : q_dot_ik(params_.dof);
    base_vel_msg.linear.y = (std::fabs(q_dot_ik(params_.dof+1)) < min_vel_lin_base_) ? 0.0 : q_dot_ik(params_.dof+1);
    base_vel_msg.linear.z = (std::fabs(q_dot_ik(params_.dof+2)) < min_vel_lin_base_) ? 0.0 : q_dot_ik(params_.dof+2);
    base_vel_msg.angular.x = (std::fabs(q_dot_ik(params_.dof+3)) < min_vel_rot_base_) ? 0.0 : q_dot_ik(params_.dof+3);
    base_vel_msg.angular.y = (std::fabs(q_dot_ik(params_.dof+4)) < min_vel_rot_base_) ? 0.0 : q_dot_ik(params_.dof+4);
    base_vel_msg.angular.z = (std::fabs(q_dot_ik(params_.dof+5)) < min_vel_rot_base_) ? 0.0 : q_dot_ik(params_.dof+5);

    base_vel_pub_.publish(base_vel_msg);
}
/* END KinematicExtensionBaseActive **********************************************************************************************/

