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
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <controller_manager_msgs/SwitchController.h>

ros::ServiceClient g_halt_client;
ros::ServiceClient g_recover_client;
ros::ServiceClient g_switch_client;
ros::Timer g_halt_timer;
ros::Timer g_silence_timer;
ros::Time g_last_received;
ros::Duration g_timeout;
double g_threshold;
bool g_recovered;
std::vector<std::string> g_controller_spawn;


void switch_controller(){
    controller_manager_msgs::SwitchController switch_srv;
    switch_srv.request.start_controllers = g_controller_spawn;
    switch_srv.request.strictness = 1;  // BEST_EFFORT
    //switch_srv.request.strictness = 2;  // STRICT

    if (g_switch_client.call(switch_srv))
    {
        if (switch_srv.response.ok)
        {
            ROS_INFO("Successfully switched controllers");
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
}

void recover(){
    std_srvs::Trigger srv;
    ROS_ERROR("RECOVER");
    g_recover_client.call(srv);
    switch_controller();
    g_recovered=srv.response.success;
}


void commandsCallback(const geometry_msgs::Twist::ConstPtr& msg){
    g_last_received = ros::Time::now();

    if(fabs(msg->linear.x)<g_threshold &&
       fabs(msg->linear.y)<g_threshold &&
       fabs(msg->linear.z)<g_threshold &&
       fabs(msg->angular.x)<g_threshold &&
       fabs(msg->angular.y)<g_threshold &&
       fabs(msg->angular.z)<g_threshold){
        if(g_recovered){
            ROS_WARN_THROTTLE(1.0, "stopped, starting halt_timer");
            g_halt_timer.start();
        }else{
            ROS_DEBUG_THROTTLE(1.0, "stopped - but not active");
        }
    }else{
        ROS_DEBUG_THROTTLE(1.0, "moving...");
        g_halt_timer.stop();
        if(!g_recovered){
            recover();
        }
    }
}

void haltCallback(const ros::TimerEvent&){
    g_halt_timer.stop();  //prevent another call
    std_srvs::Trigger srv;
    ROS_ERROR("HALT");
    g_halt_client.call(srv);
    g_recovered=false;
}

void silenceCallback(const ros::TimerEvent&){
    if(ros::Time::now()-g_last_received>g_timeout){
        if(g_recovered){
            ROS_WARN_THROTTLE(1.0, "silence - calling haltCallback");
            haltCallback(ros::TimerEvent());
        }else{
            ROS_DEBUG_THROTTLE(1.0, "silence - but not active");
        }
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cob_stop_detector");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    double timeout;
    if(!nh_priv.getParam("timeout", timeout) || timeout <= 0.0){
        ROS_ERROR("Please provide valid timeout");
        return 1;
    }

    if(!nh_priv.getParam("threshold", g_threshold)){
        ROS_ERROR("Please provide stop threshold");
        return 1;
    }

    g_recovered = true;
    g_last_received = ros::Time::now();
    g_timeout = ros::Duration(timeout);
    g_controller_spawn = {"joint_state_controller", "twist_controller", "odometry_controller"};

    g_halt_client = nh.serviceClient<std_srvs::Trigger>("driver/halt");
    g_recover_client = nh.serviceClient<std_srvs::Trigger>("driver/recover");
    g_switch_client = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
    g_halt_client.waitForExistence();
    g_recover_client.waitForExistence();
    g_switch_client.waitForExistence();
    g_halt_timer = nh.createTimer(g_timeout, haltCallback, true, false); //oneshot=true, auto_start=false
    g_silence_timer = nh.createTimer(ros::Duration(0.1), silenceCallback, false, true); //oneshot=false, auto_start=true
    ros::Subscriber command_sub = nh.subscribe("twist_controller/command", 10, commandsCallback);

    ros::spin();
    return 0;
}
