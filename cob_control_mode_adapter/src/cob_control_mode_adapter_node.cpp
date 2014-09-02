#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <brics_actuator/JointVelocities.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>


class CobControlModeAdapter
{
  public:
    void initialize()
    {
      bool success = false;
      
      joint_names_.clear();
      current_controller_names_.clear();
      vel_controller_names_.clear();
      traj_controller_names_.clear();
      vel_controller_pubs_.clear();
      
      current_control_mode_="NONE";
      
      //Get joint names from parameter server
      std::string param="joint_names";
      XmlRpc::XmlRpcValue jointNames_XMLRPC;
      if (nh_.hasParam(param))
      {
        nh_.getParam(param, jointNames_XMLRPC);
      }
      else
      {
        ROS_ERROR("Parameter %s not set, shutting down node...", param.c_str());
        nh_.shutdown();
      }

      for (unsigned int i=0; i<jointNames_XMLRPC.size(); i++)
      {
        joint_names_.push_back(static_cast<std::string>(jointNames_XMLRPC[i]));  
        ROS_INFO("JointNames: %s", joint_names_[i].c_str());
        
        ros::Publisher pub = nh_.advertise<std_msgs::Float64>(joint_names_[i]+"_velocity_controller/command", 1);
        vel_controller_pubs_.push_back(pub);
        vel_controller_names_.push_back(joint_names_[i]+"_velocity_controller");
      }
      
      param="max_vel_command_silence";
      if (nh_.hasParam(param))
      {
        nh_.getParam(param, max_vel_command_silence_);
      }
      else
      {
        ROS_ERROR("Parameter %s not set, using default 0.5s...", param.c_str());
        max_vel_command_silence_=0.5;
      }
      
      
      
      traj_controller_names_.push_back(nh_.getNamespace());
      
      ROS_INFO("Waiting for Service 'load_controller'...");
      ros::service::waitForService("/controller_manager/load_controller");
      ROS_INFO("..Service available");
      
      ROS_INFO("Waiting for Service 'switch_controller'...");
      ros::service::waitForService("/controller_manager/switch_controller");
      ROS_INFO("..Service available");
      
      load_client_ = nh_.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
      switch_client_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
      
      for (unsigned int i=0; i<traj_controller_names_.size(); i++)
      {
        success = load_controller(traj_controller_names_[i]);
      }
      
      for (unsigned int i=0; i<vel_controller_names_.size(); i++)
      {
        success = load_controller(vel_controller_names_[i]);
      }
      
      //start trajectory controller by default
      success = switch_controller(traj_controller_names_, current_controller_names_);
      current_control_mode_="TRAJECTORY";
      
      cmd_vel_sub_ = nh_.subscribe("command_vel", 1, &CobControlModeAdapter::cmd_vel_cb, this);
      
      update_rate = 100;  //[hz]
      timer = nh_.createTimer(ros::Duration(1/update_rate), &CobControlModeAdapter::update, this);
    }
    
    bool load_controller(std::string load_controller)
    {
      controller_manager_msgs::LoadController load_srv;
      load_srv.request.name = load_controller;
      if(load_client_.call(load_srv))
      {
        if(load_srv.response.ok)
        {
          ROS_INFO("%s loaded", load_controller.c_str());
          return true;
        }
        else
          ROS_ERROR("Could not load controllers");
      }
      else
        ROS_ERROR("ServiceCall failed: load_controller");
        
      return false;
    }
    
    
    bool switch_controller(std::vector< std::string > start_controllers, std::vector< std::string > stop_controllers)
    {
      controller_manager_msgs::SwitchController switch_srv;
      switch_srv.request.start_controllers = start_controllers;
      switch_srv.request.stop_controllers = stop_controllers;
      switch_srv.request.strictness = 2; //STRICT
      if(switch_client_.call(switch_srv))
      {
        if(switch_srv.response.ok)
        {
          ROS_INFO("Switched Controllers");
          current_controller_names_=start_controllers;
          return true;
        }
        else
          ROS_ERROR("Could not switch controllers");
      }
      else
        ROS_ERROR("ServiceCall failed: switch_controller");
        
      return false;
    }
    
    void cmd_vel_cb(const brics_actuator::JointVelocities::ConstPtr& msg)
    {
      if(current_control_mode_!="VELOCITY")
      {
        bool success = switch_controller(vel_controller_names_, current_controller_names_);
        if(!success)
        {
          ROS_ERROR("Unable to switch to velocity_controllers. Not executing command_vel...");
          return;
        }
      }
      for(unsigned int i=0; i<=joint_names_.size(); i++)
      {
        std_msgs::Float64 cmd;
        cmd.data= msg->velocities[i].value;
        vel_controller_pubs_[i].publish(cmd);
      }
    }

    void update(const ros::TimerEvent& event)
    {
      ros::Duration period = event.current_real - last_vel_command_;
      if(current_control_mode_!="TRAJECTORY" && period.toSec() >= max_vel_command_silence_)
      {
        bool success = switch_controller(traj_controller_names_, current_controller_names_);
        if(success)
        {
          ROS_INFO("Have not heard a vel command for %f seconds, switched back to trajectory_controller", period.toSec());
          current_control_mode_="TRAJECTORY";
        }
      }
    }

  private:
    ros::NodeHandle nh_;
    
    ros::Timer timer;
    double update_rate;
    
    std::vector< std::string > joint_names_;
    
    std::string current_control_mode_;
    std::vector< std::string > current_controller_names_;
    std::vector< std::string > vel_controller_names_;
    std::vector< std::string >  traj_controller_names_;
    
    std::vector< ros::Publisher > vel_controller_pubs_;
    ros::Subscriber cmd_vel_sub_;
    
    ros::ServiceClient load_client_;
    ros::ServiceClient switch_client_;
    
    double max_vel_command_silence_;
    ros::Time last_vel_command_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cob_control_mode_adapter_node");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  CobControlModeAdapter* ccma = new CobControlModeAdapter();
  ccma->initialize();

  ros::waitForShutdown();
  return 0;
}




