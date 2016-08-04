#include <std_srvs/Trigger.h>
#include <cob_omni_drive_controller/WheelCommands.h>
#include <ros/ros.h>

ros::ServiceClient g_halt_client;
std::vector<ros::Time> g_last_ok;
double g_threshold;
ros::Duration g_timeout;
bool g_halt_requested;

void commandsCallback(const cob_omni_drive_controller::WheelCommands::ConstPtr& msg){
    g_last_ok.resize(msg->steer_target_error.size(), msg->header.stamp);

    bool valid = true;
    for(size_t i = 0; i < msg->steer_target_error.size(); ++i){

        if(fabs(msg->steer_target_error[i]) >= g_threshold){
            valid = false;
            if( (msg->header.stamp - g_last_ok[i]) >= g_timeout  && !g_halt_requested) {
                g_halt_requested = true;
                std_srvs::Trigger srv;
                ROS_ERROR_STREAM("Wheel " << i << " exceeded threshold for too long, halting..");
                g_halt_client.call(srv);
            }
        }else{
            g_last_ok[i] = msg->header.stamp;
        }
    }

    if(valid) g_halt_requested = false;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cob_omni_drive_stuck_detector");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    double timeout;
    if(!nh_priv.getParam("timeout", timeout) || timeout <= 0.0){
        ROS_ERROR("Please provide valid timeout");
        return 1;
    }

    if(!nh_priv.getParam("threshold", g_threshold)){
        ROS_ERROR("Please provide stuck threshold");
        return 1;
    }

    g_halt_requested = false;
    g_timeout = ros::Duration(timeout);

    ros::Subscriber status_sub = nh.subscribe("twist_controller/wheel_commands", 10, commandsCallback);
    g_halt_client = nh.serviceClient<std_srvs::Trigger>("driver/halt");

    ros::spin();
    return 0;
}
