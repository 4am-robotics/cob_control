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
#ifndef KINEMATIC_EXTENSION_H_
#define KINEMATIC_EXTENSION_H_

#include "cob_twist_controller/kinematic_extensions/kinematic_extension_base.h"


#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <Eigen/Geometry>



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
        KinematicExtensionNone(const TwistControllerParams& params)
        : KinematicExtensionBase(params)
        {
            //nothing to do
        }

        ~KinematicExtensionNone() {}

        virtual KDL::Jacobian adjustJacobian(const KDL::Jacobian& jac_chain);
        virtual void processResultExtension(const KDL::JntArray& q_dot_ik);
};
/* END KinematicExtensionNone **********************************************************************************************/


/* BEGIN KinematicExtension6D ****************************************************************************************/
/// Abstract Helper Class to be used for Cartesian KinematicExtensions based on enabled DoFs.
class KinematicExtension6D : public KinematicExtensionBase
{
    public:
        KinematicExtension6D(const TwistControllerParams& params)
        : KinematicExtensionBase(params)
        {
            //nothing to do here
        }

        ~KinematicExtension6D() {}

        virtual KDL::Jacobian adjustJacobian(const KDL::Jacobian& jac_chain);
        virtual void processResultExtension(const KDL::JntArray& q_dot_ik) = 0;
        
        KDL::Jacobian adjustJacobian6d(const KDL::Jacobian& jac_chain, const KDL::Frame full_frame, const KDL::Frame partial_frame, const ActiveCartesianDimension active_dim);
};
/* END KinematicExtension6D **********************************************************************************************/


/* BEGIN KinematicExtensionBaseActive ****************************************************************************************/
/// Class implementing the a mobile base KinematicExtension with Cartesian DoFs (lin_x, lin_y, rot_z) enabled (i.e. 2D).
class KinematicExtensionBaseActive : public KinematicExtension6D
{
    public:
        KinematicExtensionBaseActive(const TwistControllerParams& params)
        : KinematicExtension6D(params)
        {
            base_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base/twist_controller/command", 1);
            
            //ToDo: I don't like this hack! Can we pass the tf_listner_ somehow?
            ///give tf_listener_ some time to fill buffer
            ros::Duration(0.5).sleep();
        }

        ~KinematicExtensionBaseActive() {}

        virtual KDL::Jacobian adjustJacobian(const KDL::Jacobian& jac_chain);
        virtual void processResultExtension(const KDL::JntArray& q_dot_ik);
        
        
        void baseTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);
        
    
    private:
        ros::NodeHandle nh_;
        tf::TransformListener tf_listener_;
    
        ros::Publisher base_vel_pub_;
};
/* END KinematicExtensionBaseActive **********************************************************************************************/

#endif /* KINEMATIC_EXTENSION_H_ */
