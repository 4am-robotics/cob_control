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


#include <string>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include <Eigen/Dense>
#include <kdl_conversions/kdl_msg.h>

class DebugEvaluateJointStates
{
    ros::NodeHandle nh_;
    ros::Subscriber jointstate_sub_;
    ros::Publisher manipulability_pub_;
    ros::Publisher twist_current_pub_;

    std::string chain_base_link_;
    std::string chain_tip_link_;

    KDL::Chain chain_;
    KDL::ChainFkSolverVel_recursive* p_fksolver_vel_;
    KDL::ChainJntToJacSolver* p_jnt2jac_;

public:
    int init()
    {
        if (!nh_.getParam("chain_base_link", this->chain_base_link_))
        {
            ROS_ERROR("Failed to get parameter \"chain_base_link\".");
            return -1;
        }

        if (!nh_.getParam("chain_tip_link", this->chain_tip_link_))
        {
            ROS_ERROR("Failed to get parameter \"chain_tip_link\".");
            return -2;
        }

        /// parse robot_description and generate KDL chains
        KDL::Tree my_tree;
        if (!kdl_parser::treeFromParam("/robot_description", my_tree))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return -3;
        }

        my_tree.getChain(this->chain_base_link_, this->chain_tip_link_, chain_);
        if (chain_.getNrOfJoints() == 0)
        {
            ROS_ERROR("Failed to initialize kinematic chain");
            return -4;
        }

        p_fksolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);
        p_jnt2jac_ = new KDL::ChainJntToJacSolver(chain_);

        /// initialize ROS interfaces
        jointstate_sub_ = nh_.subscribe("joint_states", 1, &DebugEvaluateJointStates::jointstateCallback, this);
        manipulability_pub_ = nh_.advertise<std_msgs::Float64> ("debug/manipulability", 1);
        twist_current_pub_ = nh_.advertise<geometry_msgs::Twist> ("debug/twist_current", 1);

        return 0;
    }

    void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        KDL::JntArray q = KDL::JntArray(chain_.getNrOfJoints());
        KDL::JntArray q_dot = KDL::JntArray(chain_.getNrOfJoints());

        for (unsigned int i = 0; i < msg->name.size(); i++)
        {
            q(i) = msg->position[i];
            q_dot(i) = msg->velocity[i];
        }

        /// compute current twist
        KDL::FrameVel FrameVel;
        KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(q, q_dot);
        if (p_fksolver_vel_->JntToCart(jntArrayVel, FrameVel, -1) >= 0)
        {
            geometry_msgs::Twist twist_msg;
            tf::twistKDLToMsg(FrameVel.GetTwist(), twist_msg);
            twist_current_pub_.publish(twist_msg);
        }

        /// compute manipulability
        KDL::Jacobian jac(chain_.getNrOfJoints());
        p_jnt2jac_->JntToJac(q, jac);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> prod = jac.data * jac.data.transpose();
        double d = prod.determinant();
        double kappa = std::sqrt(std::abs(d));
        std_msgs::Float64 manipulability_msg;
        manipulability_msg.data = kappa;
        manipulability_pub_.publish(manipulability_msg);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "debug_evaluate_jointstates_node");

    DebugEvaluateJointStates dejs;
    if (dejs.init() != 0)
    {
        ROS_ERROR("Failed to initialize DebugEvaluateJointStates.");
        return -1;
    }

    ros::spin();
}
