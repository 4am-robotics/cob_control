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
