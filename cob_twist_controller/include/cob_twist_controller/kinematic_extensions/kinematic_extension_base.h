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
#ifndef KINEMATIC_EXTENSION_INTERFACE_H
#define KINEMATIC_EXTENSION_INTERFACE_H

#include "ros/ros.h"

#include "cob_twist_controller/cob_twist_controller_data_types.h"

/// Base class for kinematic extensions.
class KinematicExtensionBase
{
    public:
        KinematicExtensionBase(const InvDiffKinSolverParams &params):
            params_(params)
        {}
        
        virtual ~KinematicExtensionBase() {}
        
        virtual KDL::Jacobian adjust_jacobian(const KDL::Jacobian& jac_chain) = 0;
        virtual void process_result_extension(const KDL::JntArray &q_dot_ik) = 0;
    
    protected:
        const InvDiffKinSolverParams &params_;

};


#endif /* KINEMATIC_EXTENSION_INTERFACE_H */
