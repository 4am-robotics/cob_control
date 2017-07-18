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


#ifndef COB_TWIST_CONTROLLER_COB_TWIST_CONTROLLER_H
#define COB_TWIST_CONTROLLER_COB_TWIST_CONTROLLER_H

#include <ros/ros.h>

#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.h>

#include <cob_twist_controller/TwistControllerConfig.h>
#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include <cob_twist_controller/inverse_differential_kinematics_solver.h>
#include "cob_twist_controller/controller_interfaces/controller_interface_base.h"
#include "cob_twist_controller/callback_data_mediator.h"

class CobTwistController
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber jointstate_sub_;

    ros::Subscriber twist_sub_;
    ros::Subscriber twist_stamped_sub_;

    ros::Subscriber odometry_sub_;

    ros::Publisher twist_direction_pub_;

    ros::ServiceClient register_link_client_;
    ros::Subscriber obstacle_distance_sub_;

    KDL::Chain chain_;
    JointStates joint_states_;
    KDL::Twist twist_odometry_cb_;

    TwistControllerParams twist_controller_params_;

    boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jntToCartSolver_vel_;
    boost::shared_ptr<InverseDifferentialKinematicsSolver> p_inv_diff_kin_solver_;
    boost::shared_ptr<cob_twist_controller::ControllerInterfaceBase> controller_interface_;
    boost::shared_ptr<pluginlib::ClassLoader<cob_twist_controller::ControllerInterfaceBase> > interface_loader_;

    CallbackDataMediator callback_data_mediator_;

    tf::TransformListener tf_listener_;

public:
    CobTwistController()
    {
    }

    ~CobTwistController()
    {
        this->jntToCartSolver_vel_.reset();
        this->p_inv_diff_kin_solver_.reset();
        this->controller_interface_.reset();
        this->reconfigure_server_.reset();
    }

    bool initialize();

    bool registerCollisionLinks();

    void reconfigureCallback(cob_twist_controller::TwistControllerConfig& config, uint32_t level);
    void checkSolverAndConstraints(cob_twist_controller::TwistControllerConfig& config);
    void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void solveTwist(KDL::Twist twist);
    void visualizeTwist(KDL::Twist twist);

    boost::recursive_mutex reconfig_mutex_;
    boost::shared_ptr< dynamic_reconfigure::Server<cob_twist_controller::TwistControllerConfig> > reconfigure_server_;
};

#endif  // COB_TWIST_CONTROLLER_COB_TWIST_CONTROLLER_H
