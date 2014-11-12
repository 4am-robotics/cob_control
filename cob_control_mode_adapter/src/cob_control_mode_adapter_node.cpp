#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
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
      traj_controller_names_.clear();
      pos_controller_names_.clear();
      vel_controller_names_.clear();
      
      current_control_mode_="NONE";
      
      //wait for services from controller manager
      ROS_INFO("Waiting for Service 'load_controller'...");
      ros::service::waitForService("controller_manager/load_controller", ros::Duration(5.0));
      if(ros::service::exists("controller_manager/load_controller", false))
      {
        load_client_ = nh_.serviceClient<controller_manager_msgs::LoadController>("controller_manager/load_controller");
      }
      else
      {
        ROS_ERROR("...Load service not available!");
        return;
      }
      
      ROS_INFO("Waiting for Service 'switch_controller'...");
      ros::service::waitForService("controller_manager/switch_controller", ros::Duration(5.0));
      if(ros::service::exists("controller_manager/switch_controller", false))
      {
        switch_client_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
      }
      else
      {
        ROS_ERROR("...Load service not available!");
        return;
      }
      
      std::string param="max_command_silence";
      if (nh_.hasParam(param))
      {
        nh_.getParam(param, max_command_silence_);
      }
      else
      {
        ROS_ERROR("Parameter %s not set, using default 0.5s...", param.c_str());
        max_command_silence_=0.5;
      }
      
      // List of controlled joints
      param="joint_names";
      if(!nh_.getParam(param, joint_names_))
      {
        ROS_ERROR("Parameter %s not set, shutting down node...", param.c_str());
        nh_.shutdown();
      }
      
      traj_controller_names_.push_back("joint_trajectory_controller");
      pos_controller_names_.push_back("joint_group_position_controller");
      vel_controller_names_.push_back("joint_group_velocity_controller");
      
      //load all required controllers
      for (unsigned int i=0; i<traj_controller_names_.size(); i++)
      {
        success = load_controller(traj_controller_names_[i]);
      }
      
      for (unsigned int i=0; i<pos_controller_names_.size(); i++)
      {
        success = load_controller(pos_controller_names_[i]);
      }
      
      for (unsigned int i=0; i<vel_controller_names_.size(); i++)
      {
        success = load_controller(vel_controller_names_[i]);
      }
      
      //start trajectory controller by default
      success = switch_controller(traj_controller_names_, current_controller_names_);
      current_control_mode_="TRAJECTORY";
      
      ////start velocity controllers
      //success = switch_controller(vel_controller_names_, current_controller_names_);
      //current_control_mode_="VELOCITY";
      
      cmd_pos_sub_ = nh_.subscribe("joint_group_position_controller/command", 1, &CobControlModeAdapter::cmd_pos_cb, this);
      cmd_vel_sub_ = nh_.subscribe("joint_group_velocity_controller/command", 1, &CobControlModeAdapter::cmd_vel_cb, this);
      
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
    
    void cmd_pos_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      last_pos_command_=ros::Time::now();
      if(current_control_mode_!="POSITION")
      {
        bool success = switch_controller(pos_controller_names_, current_controller_names_);
        if(!success)
        {
          ROS_ERROR("Unable to switch to position_controllers. Not executing command...");
          return;
        }
        else
        {
          ROS_INFO("Successfully switched to position_controllers");
          current_control_mode_="POSITION";
        }
      }
    }
    
    void cmd_vel_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      last_vel_command_=ros::Time::now();
      if(current_control_mode_!="VELOCITY")
      {
        bool success = switch_controller(vel_controller_names_, current_controller_names_);
        if(!success)
        {
          ROS_ERROR("Unable to switch to velocity_controllers. Not executing command...");
          return;
        }
        else
        {
          ROS_INFO("Successfully switched to velocity_controllers");
          current_control_mode_="VELOCITY";
        }
      }
    }
    
    
    

    void update(const ros::TimerEvent& event)
    {
      ros::Duration period_vel = event.current_real - last_vel_command_;
      if(current_control_mode_!="TRAJECTORY" && period_vel.toSec() >= max_command_silence_)
      {
        bool success = switch_controller(traj_controller_names_, current_controller_names_);
        if(success)
        {
          ROS_INFO("Have not heard a vel command for %f seconds, switched back to trajectory_controller", period_vel.toSec());
          current_control_mode_="TRAJECTORY";
        }
      }
      ros::Duration period_pos = event.current_real - last_pos_command_;
      if(current_control_mode_!="TRAJECTORY" && period_pos.toSec() >= max_command_silence_)
      {
        bool success = switch_controller(traj_controller_names_, current_controller_names_);
        if(success)
        {
          ROS_INFO("Have not heard a pos command for %f seconds, switched back to trajectory_controller", period_pos.toSec());
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
    std::vector< std::string > traj_controller_names_;
    std::vector< std::string > pos_controller_names_;
    std::vector< std::string > vel_controller_names_;

    ros::Subscriber cmd_pos_sub_;
    ros::Subscriber cmd_vel_sub_;
    
    ros::ServiceClient load_client_;
    ros::ServiceClient switch_client_;
    
    double max_command_silence_;
    ros::Time last_pos_command_;
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




