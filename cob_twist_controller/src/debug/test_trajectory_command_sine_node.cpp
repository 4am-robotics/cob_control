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


#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kdl/jntarray.hpp>
#include <cob_twist_controller/utils/simpson_integrator.h>

class TrajectoryCommandExecutionTester
{
public:
    TrajectoryCommandExecutionTester()
    {
        dof_ = 2;
        idx_ = 0;

        q_.resize(dof_);
        KDL::SetToZero(q_);
        q_dot_.resize(dof_);
        KDL::SetToZero(q_dot_);
        simpson_q_.resize(dof_);
        KDL::SetToZero(simpson_q_);
        euler_q_.resize(dof_);
        KDL::SetToZero(euler_q_);
        derived_simpson_q_dot_.resize(dof_);
        KDL::SetToZero(derived_simpson_q_dot_);

        integrator_.reset(new SimpsonIntegrator(dof_));
        trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory_controller/command", 1);

        output_q_pub_ = nh_.advertise<std_msgs::Float64>("integrator_debug/q", 1);
        output_q_dot_pub_ = nh_.advertise<std_msgs::Float64>("integrator_debug/q_dot", 1);
        output_simpson_q_pub_ = nh_.advertise<std_msgs::Float64>("integrator_debug/simpson_q", 1);
        output_euler_q_pub_ = nh_.advertise<std_msgs::Float64>("integrator_debug/euler_q", 1);
        output_derived_simpson_q_dot_pub_ = nh_.advertise<std_msgs::Float64>("integrator_debug/derived_simpson_q_dot", 1);

        ros::Duration(1.0).sleep();
    }


    ~TrajectoryCommandExecutionTester()
    {}

    void run()
    {
        ros::Rate r(100.0);

        ros::Time time = ros::Time::now();
        ros::Time last_update_time = time;
        ros::Duration period = time - last_update_time;
        ros::Time start_time = time;
        double x = time.toSec() - start_time.toSec();
        double old_pos = -99;

        // double a = 0.6, b = 1.0, c = 0, d = 0;    // lwa4d
        double a = 0.6, b = 0.4, c = 0, d = 0;       // torso_2dof
        euler_q_(idx_) = a*sin(b*x+c) + d;           // correct initial value

        trajectory_msgs::JointTrajectoryPoint traj_point;
        traj_point.positions.assign(dof_, 0.0);
        // traj_point.velocities.assign(dof_,0.0);

        std::vector<std::string> joint_names;
        // joint_names.push_back("arm_1_joint");
        // joint_names.push_back("arm_2_joint");
        // joint_names.push_back("arm_3_joint");
        // joint_names.push_back("arm_4_joint");
        // joint_names.push_back("arm_5_joint");
        // joint_names.push_back("arm_6_joint");
        // joint_names.push_back("arm_7_joint");
        joint_names.push_back("torso_2_joint");
        joint_names.push_back("torso_3_joint");

        while (ros::ok())
        {
            time = ros::Time::now();
            period = time - last_update_time;
            last_update_time = time;
            x = time.toSec() - start_time.toSec();

            std::vector<double> next_q;
            std::vector<double> next_q_dot;

            q_(idx_)     = a*sin(b*x+c) + d;
            q_dot_(idx_) = a*b*cos(b*x+c);

            if (integrator_->updateIntegration(q_dot_, q_, next_q, next_q_dot))
            {
                simpson_q_(idx_) = next_q[idx_];
                // q_dot_(idx_) = next_q_dot[idx_];
            }

            euler_q_(idx_) += q_dot_(idx_) * period.toSec();

            traj_point.positions[idx_] = q_(idx_);
            // traj_point.velocities[idx_] = q_dot_(idx_);
            // traj_point.time_from_start = ros::Duration(0.5);             // should be as small as possible
            traj_point.time_from_start = ros::Duration(period.toSec());     // seems to be a good value
            // traj_point.time_from_start = ros::Duration(0.1 * period.toSec());  // does not make a difference anymore
            ////// time_from_start should be as small as possible, however, setting it to none is not possible (trajectory point being dropped by controller due to occuring in the past)

            trajectory_msgs::JointTrajectory traj_msg;
            traj_msg.points.push_back(traj_point);
            traj_msg.joint_names = joint_names;
            // traj_msg.header.stamp = ros::Time::now();     //now or none - does not make a difference


            std_msgs::Float64 q_msg;       // cos
            q_msg.data = q_(idx_);
            std_msgs::Float64 q_dot_msg;   // sin
            q_dot_msg.data = q_dot_(idx_);
            std_msgs::Float64 simpson_q_msg;
            simpson_q_msg.data = simpson_q_(idx_);
            std_msgs::Float64 euler_q_msg;
            euler_q_msg.data = euler_q_(idx_);
            std_msgs::Float64 derived_simpson_q_dot_msg;
            derived_simpson_q_dot_msg.data = derived_simpson_q_dot_(idx_);

            output_q_pub_.publish(q_msg);
            output_q_dot_pub_.publish(q_dot_msg);
            output_simpson_q_pub_.publish(simpson_q_msg);
            output_euler_q_pub_.publish(euler_q_msg);
            output_derived_simpson_q_dot_pub_.publish(derived_simpson_q_dot_msg);

            trajectory_pub_.publish(traj_msg);

            ros::spinOnce();
            r.sleep();
        }
    }


    ros::NodeHandle nh_;
    ros::Publisher trajectory_pub_;

    ros::Publisher output_q_pub_;
    ros::Publisher output_q_dot_pub_;
    ros::Publisher output_simpson_q_pub_;
    ros::Publisher output_euler_q_pub_;
    ros::Publisher output_derived_simpson_q_dot_pub_;

    KDL::JntArray q_;
    KDL::JntArray q_dot_;
    KDL::JntArray simpson_q_;
    KDL::JntArray euler_q_;
    KDL::JntArray derived_simpson_q_dot_;

    boost::shared_ptr<SimpsonIntegrator> integrator_;

    unsigned int dof_;
    unsigned int idx_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_trajectory_command_execution_node");

    TrajectoryCommandExecutionTester tcet;
    tcet.run();
    ros::spin();
    return 0;
}
