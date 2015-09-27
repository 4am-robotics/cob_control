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
#ifndef KINEMATIC_EXTENSION_URDF_H_
#define KINEMATIC_EXTENSION_URDF_H_

#include <ros/ros.h>

#include <Eigen/Geometry>

#include "cob_twist_controller/kinematic_extensions/kinematic_extension_base.h"


/* BEGIN KinematicExtensionURDF ****************************************************************************************/
/// Abstract Helper Class to be used for Cartesian KinematicExtensions based on URDF.
class KinematicExtensionURDF : public KinematicExtensionBase
{
    public:
        KinematicExtensionURDF(const TwistControllerParams& params)
        : KinematicExtensionBase(params)
        {
            //nothing to do here
        }

        ~KinematicExtensionURDF() {}

        virtual KDL::Jacobian adjustJacobian(const KDL::Jacobian& jac_chain);
        virtual void processResultExtension(const KDL::JntArray& q_dot_ik) = 0;

        KDL::Jacobian adjustJacobianUrdf(const KDL::Jacobian& jac_chain, const KDL::Frame full_frame, const KDL::Frame partial_frame);
};
/* END KinematicExtensionURDF **********************************************************************************************/


#endif /* KINEMATIC_EXTENSION_URDF_H_ */
