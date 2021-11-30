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
#include <vector>
#include <algorithm>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
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

class DebugFkVelRecursive
{
    ros::NodeHandle nh_;
    ros::Subscriber joint_states_sub_;
    ros::Subscriber controller_state_sub_;
    ros::Publisher manipulability_pub_;
    ros::Publisher twist_current_pub_;
    ros::Publisher twist_magnitude_desired_pub_;
    ros::Publisher twist_magnitude_actual_pub_;

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

        //debug chain
        ROS_WARN("~~~~~~~~~~~~~ DEBUG CHAIN ~~~~~~~~~~~~~");
        ROS_WARN_STREAM("NrOfJoints: "<<chain_.getNrOfJoints());
        ROS_WARN_STREAM("NrOfSegments: "<<chain_.getNrOfSegments());
        for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++)
        {
            KDL::Segment segment = chain_.getSegment(i);
            ROS_WARN_STREAM("SegmentName: "<<segment.getName());
            KDL::Joint joint = segment.getJoint();
            ROS_WARN_STREAM("JointName: "<<joint.getName());
            ROS_WARN_STREAM("JointType: "<<joint.getTypeName());
        }
        ROS_WARN("~~~~~~~~~~~~~ DEBUG CHAIN ~~~~~~~~~~~~~");

        p_fksolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);
        p_jnt2jac_ = new KDL::ChainJntToJacSolver(chain_);

        /// initialize ROS interfaces
        // joint_states_sub_ = nh_.subscribe("joint_states", 1, &DebugFkVelRecursive::jointstatesCallback, this);  // analyzing controller_state instead
        controller_state_sub_ = nh_.subscribe("joint_trajectory_controller/state", 1, &DebugFkVelRecursive::controllerstateCallback, this);
        manipulability_pub_ = nh_.advertise<std_msgs::Float64> ("debug/manipulability", 1);
        twist_current_pub_ = nh_.advertise<geometry_msgs::Twist> ("debug/twist_current", 1);
        twist_magnitude_desired_pub_ = nh_.advertise<sensor_msgs::JointState> ("debug/twist_magnitude/desired", 1);
        twist_magnitude_actual_pub_ = nh_.advertise<sensor_msgs::JointState> ("debug/twist_magnitude/actual", 1);

        return 0;
    }

    void jointstatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        KDL::JntArray q = KDL::JntArray(chain_.getNrOfJoints());
        KDL::JntArray q_dot = KDL::JntArray(chain_.getNrOfJoints());

        /// extract q, q_dot from JointStates
        for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++)
        {
            KDL::Joint joint = chain_.getSegment(i).getJoint();
            std::string joint_name = joint.getName();
            if (joint.getType() == KDL::Joint::None)
            {
                ROS_DEBUG_STREAM("Skip fixed Joint '" << joint_name);
                continue;
            }
            else
            {
                if (std::find(msg->name.begin(), msg->name.end(), joint_name) != msg->name.end())
                {
                    unsigned int index = std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), joint_name));
                    q(index) = msg->position[index];
                    q_dot(index) = msg->velocity[index];
                }
                else
                {
                    ROS_ERROR_STREAM("Joint '" << joint_name << "' not found in JointStates");
                    return;
                }
            }
        }

        /// compute current twist
        std::vector< KDL::FrameVel > v_FrameVel;
        KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(q, q_dot);
        v_FrameVel.resize(chain_.getNrOfSegments());
        if (p_fksolver_vel_->JntToCart(jntArrayVel, v_FrameVel, chain_.getNrOfSegments()) >= 0)
        {
            //last entry is twist of tip_link
            geometry_msgs::Twist twist_msg;
            tf::twistKDLToMsg(v_FrameVel.back().GetTwist(), twist_msg);
            twist_current_pub_.publish(twist_msg);

            //recursively calculate FrameVel magnitudes
            sensor_msgs::JointState magnitude_msg;
            magnitude_msg.header.stamp = msg->header.stamp;
            for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++)
            {
                magnitude_msg.name.push_back(chain_.getSegment(i).getName());
                magnitude_msg.velocity.push_back(v_FrameVel[i].GetTwist().vel.Norm());
            }
            twist_magnitude_actual_pub_.publish(magnitude_msg);
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

    void controllerstateCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
    {
        KDL::JntArray q_desired = KDL::JntArray(chain_.getNrOfJoints());
        KDL::JntArray q_desired_dot = KDL::JntArray(chain_.getNrOfJoints());
        KDL::JntArray q_actual = KDL::JntArray(chain_.getNrOfJoints());
        KDL::JntArray q_actual_dot = KDL::JntArray(chain_.getNrOfJoints());

        for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++)
        {
            KDL::Joint joint = chain_.getSegment(i).getJoint();
            std::string joint_name = joint.getName();
            if (joint.getType() == KDL::Joint::None)
            {
                ROS_DEBUG_STREAM("Skip fixed Joint '" << joint_name);
                continue;
            }
            else
            {
                if (std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_name) != msg->joint_names.end())
                {
                    unsigned int index = std::distance(msg->joint_names.begin(), std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_name));
                    q_desired(index) = msg->desired.positions[index];
                    q_desired_dot(index) = msg->desired.velocities[index];
                    q_actual(index) = msg->actual.positions[index];
                    q_actual_dot(index) = msg->actual.velocities[index];
                }
                else
                {
                    ROS_ERROR_STREAM("Joint '" << joint_name << "' not found in JointTrajectoryControllerState");
                    return;
                }
            }
        }

        /// compute desired twists recursively
        std::vector< KDL::FrameVel > v_FrameVel_desired;
        KDL::JntArrayVel jntArrayVel_desired = KDL::JntArrayVel(q_desired, q_desired_dot);
        v_FrameVel_desired.resize(chain_.getNrOfSegments());
        if (p_fksolver_vel_->JntToCart(jntArrayVel_desired, v_FrameVel_desired, chain_.getNrOfSegments()) >= 0)
        {
            sensor_msgs::JointState magnitude_desired_msg;
            magnitude_desired_msg.header.stamp = msg->header.stamp;
            for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++)
            {
                magnitude_desired_msg.name.push_back(chain_.getSegment(i).getName());
                magnitude_desired_msg.velocity.push_back(v_FrameVel_desired[i].GetTwist().vel.Norm());
            }
            twist_magnitude_desired_pub_.publish(magnitude_desired_msg);
        }
        /// compute actual twists recursively
        std::vector< KDL::FrameVel > v_FrameVel_actual;
        KDL::JntArrayVel jntArrayVel_actual = KDL::JntArrayVel(q_actual, q_actual_dot);
        v_FrameVel_actual.resize(chain_.getNrOfSegments());
        if (p_fksolver_vel_->JntToCart(jntArrayVel_actual, v_FrameVel_actual, chain_.getNrOfSegments()) >= 0)
        {
            sensor_msgs::JointState magnitude_actual_msg;
            magnitude_actual_msg.header.stamp = msg->header.stamp;
            for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++)
            {
                magnitude_actual_msg.name.push_back(chain_.getSegment(i).getName());
                magnitude_actual_msg.velocity.push_back(v_FrameVel_actual[i].GetTwist().vel.Norm());
            }
            twist_magnitude_actual_pub_.publish(magnitude_actual_msg);
        }
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "debug_fk_vel_recursive_node");

    DebugFkVelRecursive dfvr;
    if (dfvr.init() != 0)
    {
        ROS_ERROR("Failed to initialize DebugFkVelRecursive.");
        return -1;
    }

    ros::spin();
}
