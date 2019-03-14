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


#include <std_srvs/Trigger.h>
#include <cob_base_controller_utils/WheelCommands.h>
#include <ros/ros.h>

ros::ServiceClient g_stop_client;
std::vector<ros::Time> g_last_ok;
double g_threshold;
ros::Duration g_timeout;
bool g_stop_requested;
bool g_shutdown;

void commandsCallback(const cob_base_controller_utils::WheelCommands::ConstPtr& msg){
    g_last_ok.resize(msg->steer_target_velocity.size(), msg->header.stamp);

    bool valid = true;
    for(size_t i = 0; i < msg->steer_target_velocity.size(); ++i){

        if(fabs(msg->steer_target_velocity[i]) >= g_threshold){
            valid = false;
            if( (msg->header.stamp - g_last_ok[i]) >= g_timeout  && !g_stop_requested) {
                g_stop_requested = true;
                std_srvs::Trigger srv;
                ROS_ERROR_STREAM("Wheel " << i << " exceeded threshold for too long, stopping..");
                g_stop_client.call(srv);
            }
        }else{
            g_last_ok[i] = msg->header.stamp;
        }
    }

    if(valid) g_stop_requested = false;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cob_spin_detector");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    double timeout;
    if(!nh_priv.getParam("timeout", timeout) || timeout <= 0.0){
        ROS_ERROR("Please provide valid timeout");
        return 1;
    }

    if(!nh_priv.getParam("threshold", g_threshold)){
        ROS_ERROR("Please provide spin threshold");
        return 1;
    }

    if(!nh_priv.getParam("shutdown", g_shutdown)){
        ROS_ERROR("Please specify whether shutdown (true) or halt (false) is called");
        return 1;
    }

    g_stop_requested = false;
    g_timeout = ros::Duration(timeout);

    ros::Subscriber status_sub = nh.subscribe("twist_controller/wheel_commands", 10, commandsCallback);
    if(g_shutdown){
        g_stop_client = nh.serviceClient<std_srvs::Trigger>("driver/shutdown");
    }else{
        g_stop_client = nh.serviceClient<std_srvs::Trigger>("driver/halt");
    }

    ros::spin();
    return 0;
}
