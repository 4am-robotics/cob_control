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
 *   controller interfaces (Velocity/Position/Trajectory/JointStates).
 *
 ****************************************************************/

#ifndef COB_TWIST_CONTROLLER_CONTROLLER_INTERFACES_CONTROLLER_INTERFACE_H
#define COB_TWIST_CONTROLLER_CONTROLLER_INTERFACES_CONTROLLER_INTERFACE_H

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <boost/thread/mutex.hpp>

#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/utils/moving_average.h"

#include "cob_twist_controller/controller_interfaces/controller_interface_base.h"


namespace cob_twist_controller
{

/* BEGIN ControllerInterfaceVelocity ****************************************************************************************/
/// Class providing a ControllerInterface publishing velocities.
class ControllerInterfaceVelocity : public ControllerInterfaceBase
{
    public:
        ControllerInterfaceVelocity() {}
        ~ControllerInterfaceVelocity() {}

        virtual void initialize(ros::NodeHandle& nh,
                                const TwistControllerParams& params);
        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);
};
/* END ControllerInterfaceVelocity **********************************************************************************************/


/* BEGIN ControllerInterfacePosition ****************************************************************************************/
/// Class providing a ControllerInterface publishing JointGroupPositionCommands.
class ControllerInterfacePosition : public ControllerInterfacePositionBase
{
    public:
        ControllerInterfacePosition() {}
        ~ControllerInterfacePosition() {}

        virtual void initialize(ros::NodeHandle& nh,
                                const TwistControllerParams& params);
        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);
};
/* END ControllerInterfacePosition **********************************************************************************************/


/* BEGIN ControllerInterfaceTrajectory ****************************************************************************************/
/// Class providing a ControllerInterface publishing a JointTrajectory.
class ControllerInterfaceTrajectory : public ControllerInterfacePositionBase
{
    public:
        ControllerInterfaceTrajectory() {}
        ~ControllerInterfaceTrajectory() {}

        virtual void initialize(ros::NodeHandle& nh,
                                const TwistControllerParams& params);
        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);
};
/* END ControllerInterfaceTrajectory **********************************************************************************************/

/* BEGIN ControllerInterfaceJointStates ****************************************************************************************/
/// Class providing a ControllerInterface publishing JointStates.
class ControllerInterfaceJointStates : public ControllerInterfacePositionBase
{
    public:
        ControllerInterfaceJointStates() {}
        ~ControllerInterfaceJointStates() {}

        virtual void initialize(ros::NodeHandle& nh,
                                const TwistControllerParams& params);
        virtual void processResult(const KDL::JntArray& q_dot_ik,
                                   const KDL::JntArray& current_q);

    private:
        boost::mutex mutex_;
        sensor_msgs::JointState js_msg_;

        ros::Timer js_timer_;
        void publishJointState(const ros::TimerEvent& event);
};
/* END ControllerInterfaceJointStates **********************************************************************************************/

}

#endif  // COB_TWIST_CONTROLLER_CONTROLLER_INTERFACES_CONTROLLER_INTERFACE_H
