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
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: June, 2015
 *
 * \brief
 *   This header contains the interface description of all available
 *   hardware interface types (position/velocity).
 *
 ****************************************************************/
#ifndef HARDWARE_INTERFACE_TYPE_BASE_H_
#define HARDWARE_INTERFACE_TYPE_BASE_H_

#include "ros/ros.h"

#include "cob_twist_controller/cob_twist_controller_data_types.h"

/// Base class for hardware interfaces types.
class HardwareInterfaceBase
{
    public:
        HardwareInterfaceBase(ros::NodeHandle& nh,
                              const TwistControllerParams& params):
            nh_(nh),
            params_(params)
        {}

        virtual ~HardwareInterfaceBase() {}

        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q) = 0;


    protected:
        const TwistControllerParams& params_;
        ros::NodeHandle& nh_;
        ros::Publisher pub_;
};


#endif /* HARDWARE_INTERFACE_TYPE_BASE_H_ */
