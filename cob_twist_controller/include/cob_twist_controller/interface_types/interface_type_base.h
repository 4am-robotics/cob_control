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
 *   interface types (position/velocity).
 *
 ****************************************************************/
#ifndef COB_CONTROL_COB_TWIST_CONTROLLER_INCLUDE_INTERFACE_TYPES_INTERFACE_TYPE_BASE_H_
#define COB_CONTROL_COB_TWIST_CONTROLLER_INCLUDE_INTERFACE_TYPES_INTERFACE_TYPE_BASE_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/utils/moving_average.h"

/// Base class for interfaces types.
class InterfaceBase
{
    public:
        InterfaceBase(ros::NodeHandle& nh, const TwistControllerParams &params):
            nh_(nh),
            params_(params)
        {}

        virtual ~InterfaceBase() {}

        virtual void process_result(const KDL::JntArray &q_dot_ik,
                                    std::vector<double> &initial_position) = 0;


    protected:
        const TwistControllerParams &params_;
        ros::NodeHandle& nh_;
        ros::Publisher pub_;
};


#endif /* COB_CONTROL_COB_TWIST_CONTROLLER_INCLUDE_INTERFACE_TYPES_INTERFACE_TYPE_BASE_H_ */
