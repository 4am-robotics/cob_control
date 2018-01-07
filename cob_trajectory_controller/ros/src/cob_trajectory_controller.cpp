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


#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <cob_trajectory_controller/genericArmCtrl.h>
// ROS service includes
#include <std_srvs/Trigger.h>
#include <cob_srvs/SetString.h>
#include <cob_srvs/SetFloat.h>


#define HZ 100

class cob_trajectory_controller_node
{
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_private_;

    ros::Publisher joint_vel_pub_;
    ros::Subscriber controller_state_;
    ros::Subscriber operation_mode_;
    ros::ServiceServer srvServer_Stop_;
    ros::ServiceServer srvServer_SetVel_;
    ros::ServiceServer srvServer_SetAcc_;
    ros::ServiceClient srvClient_SetOperationMode;

    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;

    std::string action_name_;
    std::string current_operation_mode_;
    std::vector<std::string> JointNames_;
    bool executing_;
    bool failure_;
    bool rejected_;
    bool preemted_;
    int DOF;
    double velocity_timeout_;

    int watchdog_counter;
    genericArmCtrl* traj_generator_;
    trajectory_msgs::JointTrajectory traj_;
    trajectory_msgs::JointTrajectory traj_2_;
    std::vector<double> q_current, startposition_, joint_distance_;

public:

    cob_trajectory_controller_node():
    as_(n_, "joint_trajectory_controller/follow_joint_trajectory", boost::bind(&cob_trajectory_controller_node::executeFollowTrajectory, this, _1), false),
    action_name_("follow_joint_trajectory")
    {
        joint_vel_pub_ = n_.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);
        controller_state_ = n_.subscribe("joint_trajectory_controller/state", 1, &cob_trajectory_controller_node::state_callback, this);
        operation_mode_ = n_.subscribe("driver/current_operationmode", 1, &cob_trajectory_controller_node::operationmode_callback, this);
        srvServer_Stop_ = n_.advertiseService("driver/stop", &cob_trajectory_controller_node::srvCallback_Stop, this);
        srvServer_SetVel_ = n_.advertiseService("driver/set_joint_velocity", &cob_trajectory_controller_node::srvCallback_setVel, this);
        srvServer_SetAcc_ = n_.advertiseService("driver/set_joint_acceleration", &cob_trajectory_controller_node::srvCallback_setAcc, this);
        srvClient_SetOperationMode = n_.serviceClient<cob_srvs::SetString>("driver/set_operation_mode");
        while(!srvClient_SetOperationMode.exists())
        {
            ROS_INFO("Waiting for operationmode service to become available");
            sleep(1);
        }
        executing_ = false;
        failure_ = false;
        rejected_ = false;
        preemted_ = false;
        watchdog_counter = 0;
        current_operation_mode_ = "undefined";
        double PTPvel = 0.7;
        double PTPacc = 0.2;
        double maxError = 0.7;
        double overlap_time = 0.4;
        velocity_timeout_ = 2.0;
        DOF = 7;
        // get JointNames from parameter server
        ROS_INFO("getting JointNames from parameter server: %s", n_private_.getNamespace().c_str());
        if (n_private_.hasParam("joint_names"))
        {
            n_private_.getParam("joint_names", JointNames_);
        }
        else
        {
            ROS_ERROR("Parameter 'joint_names' not set");
        }
        DOF = JointNames_.size();

        if (n_private_.hasParam("ptp_vel"))
        {
            n_private_.getParam("ptp_vel", PTPvel);
        }
        if (n_private_.hasParam("ptp_acc"))
        {
            n_private_.getParam("ptp_acc", PTPacc);
        }
        if (n_private_.hasParam("max_error"))
        {
            n_private_.getParam("max_error", maxError);
        }
        if (n_private_.hasParam("overlap_time"))
        {
            n_private_.getParam("overlap_time", overlap_time);
        }
        if (n_private_.hasParam("operation_mode"))
        {
            n_private_.getParam("operation_mode", current_operation_mode_);
        }
        q_current.resize(DOF);
        ROS_INFO("starting controller with DOF: %d PTPvel: %f PTPAcc: %f maxError %f", DOF, PTPvel, PTPacc, maxError);
        traj_generator_ = new genericArmCtrl(DOF, PTPvel, PTPacc, maxError);
        traj_generator_->overlap_time = overlap_time;

        as_.start();
    }

    double getFrequency()
    {
        double frequency;
        if (n_private_.hasParam("frequency"))
        {
            n_private_.getParam("frequency", frequency);
            ROS_INFO("Setting controller frequency to %f HZ", frequency);
        }
        else
        {
            frequency = 100; //Hz
            ROS_WARN("Parameter 'frequency' not available, setting to default value: %f Hz", frequency);
        }
        return frequency;
    }

    bool srvCallback_Stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        ROS_INFO("Stopping trajectory controller.");

        // stop trajectory controller
        executing_ = false;
        res.success = true;
        traj_generator_->isMoving = false;
        failure_ = true;
        return true;
    }

    bool srvCallback_setVel(cob_srvs::SetFloat::Request &req, cob_srvs::SetFloat::Response &res)
    {
        ROS_INFO("Setting velocity to %f", req.data);
        traj_generator_->SetPTPvel(req.data);
        res.success = true;
        return true;
    }

    bool srvCallback_setAcc(cob_srvs::SetFloat::Request &req, cob_srvs::SetFloat::Response &res)
    {
        ROS_INFO("Setting acceleration to %f", req.data);
        traj_generator_->SetPTPacc(req.data);
        res.success = true;
        return true;
    }

    void operationmode_callback(const std_msgs::StringPtr& message)
    {
        current_operation_mode_ = message->data;
    }

    void state_callback(const control_msgs::JointTrajectoryControllerStatePtr& message)
    {
        std::vector<double> positions = message->actual.positions;
        for(unsigned int i = 0; i < positions.size(); i++)
        {
            q_current[i] = positions[i];
        }
    }

    void spawnTrajector(trajectory_msgs::JointTrajectory trajectory)
    {
        if(!executing_ || preemted_)
        {
            //set component to velocity mode
            cob_srvs::SetString opmode;
            opmode.request.data = "velocity";
            srvClient_SetOperationMode.call(opmode);
            ros::Time begin = ros::Time::now();
            while(current_operation_mode_ != "velocity")
            {
                ROS_INFO("waiting for component to go to velocity mode");
                usleep(100000);
                //add timeout and set action to rejected
                if((ros::Time::now() - begin).toSec() > velocity_timeout_)
                {
                    rejected_ = true;
                    return;
                }
            }

            std::vector<double> traj_start;
            if(preemted_ == true) //Calculate trajectory for runtime modification of trajectories
            {
                ROS_INFO("There is a old trajectory currently running");
                traj_start = traj_generator_->last_q;
                trajectory_msgs::JointTrajectory temp_traj;
                temp_traj = trajectory;
                //Insert the saved point as first point of trajectory, then generate SPLINE trajectory
                trajectory_msgs::JointTrajectoryPoint p;
                p.positions.resize(DOF);
                p.velocities.resize(DOF);
                p.accelerations.resize(DOF);
                for(int i = 0; i<DOF; i++)
                {
                    p.positions.at(i) = traj_start.at(i);
                    p.velocities.at(i) = 0.0;
                    p.accelerations.at(i) = 0.0;
                }
                std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator it;
                it = temp_traj.points.begin();
                temp_traj.points.insert(it,p);
                //Now insert the current as first point of trajectory, then generate SPLINE trajectory
                for(int i = 0; i<DOF; i++)
                {
                    p.positions.at(i) = traj_generator_->last_q1.at(i);
                    p.velocities.at(i) = 0.0;
                    p.accelerations.at(i) = 0.0;
                }
                it = temp_traj.points.begin();
                temp_traj.points.insert(it,p);
                for(int i = 0; i<DOF; i++)
                {
                    p.positions.at(i) = traj_generator_->last_q2.at(i);
                    p.velocities.at(i) = 0.0;
                    p.accelerations.at(i) = 0.0;
                }
                it = temp_traj.points.begin();
                temp_traj.points.insert(it,p);
                for(int i = 0; i<DOF; i++)
                {
                    p.positions.at(i) = traj_generator_->last_q3.at(i);
                    p.velocities.at(i) = 0.0;
                    p.accelerations.at(i) = 0.0;
                }
                it = temp_traj.points.begin();
                temp_traj.points.insert(it,p);
                for(int i = 0; i<DOF; i++)
                {
                    p.positions.at(i) = q_current.at(i);
                    p.velocities.at(i) = 0.0;
                    p.accelerations.at(i) = 0.0;
                }
                it = temp_traj.points.begin();
                temp_traj.points.insert(it,p);
                traj_generator_->isMoving = false ;
                traj_generator_->moveTrajectory(temp_traj, traj_start);
            }
            else //Normal calculation of trajectories
            {
                traj_start = q_current;
                trajectory_msgs::JointTrajectory temp_traj;
                temp_traj = trajectory;
                if(temp_traj.points.size() == 1)
                {
                    traj_generator_->isMoving = false ;
                    traj_generator_->moveThetas(temp_traj.points[0].positions, traj_start);
                }
                else
                {
                    //Insert the current point as first point of trajectory, then generate SPLINE trajectory
                    trajectory_msgs::JointTrajectoryPoint p;
                    p.positions.resize(DOF);
                    p.velocities.resize(DOF);
                    p.accelerations.resize(DOF);
                    for(int i = 0; i<DOF; i++)
                    {
                        p.positions.at(i) = traj_start.at(i);
                        p.velocities.at(i) = 0.0;
                        p.accelerations.at(i) = 0.0;
                    }
                    std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator it;
                    it = temp_traj.points.begin();
                    temp_traj.points.insert(it,p);
                    traj_generator_->isMoving = false ;
                    traj_generator_->moveTrajectory(temp_traj, traj_start);
                }
            }

            executing_ = true;
            startposition_ = q_current;
            preemted_ = false;

        }
        else //suspend current movement and start new one
        {
        }
        while(executing_)
        {
            if(!preemted_)
            {
                usleep(1000);
            }
            else
            {
                return;
            }
        }
    }


    void executeFollowTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
    {
        ROS_INFO("Received new goal trajectory with %lu points",goal->trajectory.points.size());
        spawnTrajector(goal->trajectory);
        // only set to succeeded if component could reach position. this is currently not the care for e.g. by emergency stop, hardware error or exceeds limit.
        if(rejected_)
            as_.setAborted(); //setRejected not implemented in simpleactionserver ?
        else
        {
            if(failure_)
                as_.setAborted();
            else
                as_.setSucceeded();
        }
        rejected_ = false;
        failure_ = false;
    }

    void run()
    {
        if(executing_)
        {
            failure_ = false;
            watchdog_counter = 0;
            if (!ros::ok() || current_operation_mode_ != "velocity")
            {
                // set the action state to preempted
                executing_ = false;
                traj_generator_->isMoving = false;
                failure_ = true;
                return;
            }
            if (as_.isPreemptRequested())
            {
                failure_ = true;
                preemted_ = true;
                executing_ = false;
                traj_generator_->isMoving = false;
                ROS_INFO("Preempted trajectory action");
                return;
            }
            std::vector<double> des_vel;
            if(traj_generator_->step(q_current, des_vel))
            {
                if(!traj_generator_->isMoving) //Finished trajectory
                {
                    executing_ = false;
                    preemted_ = false;
                }
                std_msgs::Float64MultiArray target_joint_vel;
                for(int i=0; i<DOF; i++)
                {
                    target_joint_vel.data.push_back(des_vel.at(i));
                }

                //send everything
                joint_vel_pub_.publish(target_joint_vel);
            }
            else
            {
                ROS_INFO("An controller error occured!");
                failure_ = true;
                executing_ = false;
            }
        }
        else
        {   //WATCHDOG TODO: don't always send
            if(watchdog_counter < 10)
            {
                std_msgs::Float64MultiArray target_joint_vel;
                for(int i=0; i<DOF; i++)
                {
                    target_joint_vel.data.push_back(0.0);
                }
                joint_vel_pub_.publish(target_joint_vel);
            }
            watchdog_counter++;
        }
    }

};



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "cob_trajectory_controller");

    cob_trajectory_controller_node tm;

    /// get main loop parameters
    double frequency = tm.getFrequency();

    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        tm.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
}



