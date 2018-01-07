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
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <boost/thread/mutex.hpp>

class CobControlModeAdapter
{
public:
    bool initialize()
    {
        bool success = false;

        joint_names_.clear();
        current_controller_names_.clear();
        traj_controller_names_.clear();
        pos_controller_names_.clear();
        vel_controller_names_.clear();
        last_pos_command_ = ros::Time();
        last_vel_command_ = ros::Time();

        has_traj_controller_ = false;
        has_pos_controller_ = false;
        has_vel_controller_ = false;

        current_control_mode_ = NONE;

        // wait for services from controller manager
        while (!ros::service::waitForService("controller_manager/load_controller", ros::Duration(5.0))) {;}

        if (ros::service::exists("controller_manager/load_controller", false))
        {
            load_client_ = nh_.serviceClient<controller_manager_msgs::LoadController>("controller_manager/load_controller");
        }
        else
        {
            ROS_ERROR("...Load service not available!");
            return false;
        }

        while (!ros::service::waitForService("controller_manager/switch_controller", ros::Duration(5.0))) {;}

        if (ros::service::exists("controller_manager/switch_controller", false))
        {
            switch_client_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
        }
        else
        {
            ROS_ERROR("...Load service not available!");
            return false;
        }

        std::string param = "max_command_silence";
        if (nh_.hasParam(param))
        {
            nh_.getParam(param, max_command_silence_);
        }
        else
        {
            ROS_ERROR("Parameter %s not set, using default 0.3s...", param.c_str());
            max_command_silence_ = 0.3;
        }

        // List of controlled joints
        param = "joint_names";
        if (!nh_.getParam(param, joint_names_))
        {
            ROS_ERROR("Parameter %s not set, shutting down node...", param.c_str());
            nh_.shutdown();
        }

        std::string joint_trajectory_controller_name = nh_.param<std::string>("joint_trajectory_controller_name", "joint_trajectory_controller");

        if (nh_.hasParam(joint_trajectory_controller_name))
        {
            has_traj_controller_ = true;
            traj_controller_names_.push_back(joint_trajectory_controller_name);
            ROS_DEBUG_STREAM(nh_.getNamespace() << " supports '" << joint_trajectory_controller_name << "'");
        }

        std::string joint_group_position_controller_name = nh_.param<std::string>("joint_group_position_controller_name", "joint_group_position_controller");

        if (nh_.hasParam(joint_group_position_controller_name))
        {
            has_pos_controller_ = true;
            pos_controller_names_.push_back(joint_group_position_controller_name);
            ROS_DEBUG_STREAM(nh_.getNamespace() << " supports '" << joint_group_position_controller_name << "'");
        }


        std::string joint_group_velocity_controller_name = nh_.param<std::string>("joint_group_velocity_controller_name", "joint_group_velocity_controller");

        if (nh_.hasParam(joint_group_velocity_controller_name))
        {
            has_vel_controller_ = true;
            vel_controller_names_.push_back(joint_group_velocity_controller_name);
            ROS_DEBUG_STREAM(nh_.getNamespace() << " supports '" << joint_group_velocity_controller_name << "'");
        }

        // load all required controllers
        if (has_traj_controller_)
        {
            for (unsigned int i = 0; i < traj_controller_names_.size(); i++)
            {
                success = loadController(traj_controller_names_[i]);
            }
        }
        if (has_pos_controller_)
        {
            for (unsigned int i = 0; i < pos_controller_names_.size(); i++)
            {
                success = loadController(pos_controller_names_[i]);
            }
            cmd_pos_sub_ = nh_.subscribe(joint_group_position_controller_name+"/command", 1, &CobControlModeAdapter::cmd_pos_cb, this);
        }
        if (has_vel_controller_)
        {
            for (unsigned int i = 0; i < vel_controller_names_.size(); i++)
            {
                success = loadController(vel_controller_names_[i]);
            }
            cmd_vel_sub_ = nh_.subscribe(joint_group_velocity_controller_name+"/command", 1, &CobControlModeAdapter::cmd_vel_cb, this);
        }

        // start trajectory controller by default
        if (has_traj_controller_)
        {
            success = switchController(traj_controller_names_, current_controller_names_);
            current_control_mode_ = TRAJECTORY;
        }

        update_rate_ = 100;    // [hz]
        timer_ = nh_.createTimer(ros::Duration(1/update_rate_), &CobControlModeAdapter::update, this);

        return true;
    }

    bool loadController(std::string load_controller)
    {
        controller_manager_msgs::LoadController load_srv;
        load_srv.request.name = load_controller;
        if (load_client_.call(load_srv))
        {
            if (load_srv.response.ok)
            {
                ROS_INFO("%s loaded", load_controller.c_str());
                return true;
            }
            else
            {
                ROS_ERROR("Could not load controllers");
            }
        }
        else
        {
            ROS_ERROR("ServiceCall failed: load_controller");
        }

        return false;
    }


    bool switchController(std::vector< std::string > start_controllers, std::vector< std::string > stop_controllers)
    {
        controller_manager_msgs::SwitchController switch_srv;
        switch_srv.request.start_controllers = start_controllers;
        switch_srv.request.stop_controllers = stop_controllers;
        switch_srv.request.strictness = 2;  // STRICT

        if (switch_client_.call(switch_srv))
        {
            if (switch_srv.response.ok)
            {
                std::string str_start;
                std::string str_stop;

                if (start_controllers.empty())
                {
                    str_start = "no_start_controller_defined";
                }
                else
                {
                    str_start = start_controllers.back();
                }
                if (stop_controllers.empty())
                {
                    str_stop = "no_stop_controller_defined";
                }
                else
                {
                    str_stop    = stop_controllers.back();
                }

                ROS_INFO("Switched Controllers. From %s to %s", str_stop.c_str(), str_start.c_str());

                current_controller_names_ = start_controllers;
                return true;
            }
            else
            {
                ROS_ERROR("Could not switch controllers");
            }
        }
        else
        {
            ROS_ERROR("ServiceCall failed: switch_controller");
        }

        return false;
    }

    void cmd_pos_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(mutex_);
        last_pos_command_ = ros::Time::now();
    }

    void cmd_vel_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(mutex_);
        last_vel_command_ = ros::Time::now();
    }

    void update(const ros::TimerEvent& event)
    {
        if (!nh_.ok()) {return;}

        boost::mutex::scoped_lock lock(mutex_);

        ros::Duration period_vel = event.current_real - last_vel_command_;
        ros::Duration period_pos = event.current_real - last_pos_command_;

        lock.unlock();

        if (has_vel_controller_ && (period_vel.toSec() < max_command_silence_))
        {
            if (current_control_mode_ != VELOCITY)
            {
                bool success = switchController(vel_controller_names_, current_controller_names_);
                if (!success)
                {
                    ROS_ERROR("Unable to switch to velocity_controllers. Not executing command...");
                }
                else
                {
                    ROS_INFO("Successfully switched to velocity_controllers");
                    current_control_mode_ = VELOCITY;
                }
            }
        }
        else if (has_pos_controller_ && (period_pos.toSec() < max_command_silence_))
        {
            if (current_control_mode_ != POSITION)
            {
                bool success = switchController(pos_controller_names_, current_controller_names_);
                if (!success)
                {
                    ROS_ERROR("Unable to switch to position_controllers. Not executing command...");
                }
                else
                {
                    ROS_INFO("Successfully switched to position_controllers");
                    current_control_mode_ = POSITION;
                }
            }
        }
        else if (has_traj_controller_)
        {
            if (current_control_mode_ != TRAJECTORY)
            {
                if (current_control_mode_ == POSITION)
                {
                    ROS_INFO("Have not heard a pos command for %f seconds, switched back to trajectory_controller", period_pos.toSec());
                }
                else
                {
                    ROS_INFO("Have not heard a vel command for %f seconds, switched back to trajectory_controller", period_vel.toSec());
                }
                bool success = switchController(traj_controller_names_, current_controller_names_);

                if (success)
                {
                    current_control_mode_ = TRAJECTORY;
                }
                else
                {
                    ROS_ERROR("Unable to switch to trajectory_controller. Not executing command...");
                }
            }
        }
        else
        {
            ROS_ERROR_STREAM(nh_.getNamespace() << " does not support 'joint_trajectory_controller' and no other controller was started yet");
        }
    }

private:
    ros::NodeHandle nh_;

    ros::Timer timer_;
    double update_rate_;

    std::vector< std::string > joint_names_;

    enum
    {
        NONE, VELOCITY, POSITION, TRAJECTORY
    } current_control_mode_;

    std::vector< std::string > current_controller_names_;
    std::vector< std::string > traj_controller_names_;
    std::vector< std::string > pos_controller_names_;
    std::vector< std::string > vel_controller_names_;

    bool has_traj_controller_;
    bool has_pos_controller_;
    bool has_vel_controller_;

    ros::Subscriber cmd_pos_sub_;
    ros::Subscriber cmd_vel_sub_;

    ros::ServiceClient load_client_;
    ros::ServiceClient switch_client_;

    double max_command_silence_;
    ros::Time last_pos_command_;
    ros::Time last_vel_command_;
    boost::mutex mutex_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cob_control_mode_adapter_node");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    CobControlModeAdapter* ccma = new CobControlModeAdapter();

    if (!ccma->initialize())
    {
        ROS_ERROR("Failed to initialize CobControlModeAdapter");
        return -1;
    }

    ros::waitForShutdown();
    return 0;
}

