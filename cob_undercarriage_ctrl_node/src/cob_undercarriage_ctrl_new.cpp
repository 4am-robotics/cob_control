/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_undercarriage_ctrl
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: April 2010:
 * ToDo: Adapt Interface to controller -> remove last variable (not used anymore)
 *       Rework Ctrl-class to work with SI-Units -> remove conversion
 *       For easier association of joints use platform.urdf!!
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
#include <math.h>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cob_msgs/EmergencyStopState.h>
#include <control_msgs/JointTrajectoryControllerState.h>

// external includes
#include <cob_omni_drive_controller/UndercarriageCtrlGeomROS.h>
#include <cob_omni_drive_controller/OdometryTracker.h>
#include <vector>
#include <angles/angles.h>

//####################
//#### node class ####
class NodeClass
{
  //
  public:
    // create a handle for this node, initialize node
    ros::NodeHandle n; // parameter are uploaded to private space

    // topics to publish
    ros::Publisher topic_pub_joint_state_cmd_;          // cmd issued for single joints of undercarriage
    ros::Publisher topic_pub_controller_joint_command_;
    ros::Publisher topic_pub_odometry_;                 // calculated (measured) velocity, rotation and pose (odometry-based) for the robot
    tf::TransformBroadcaster tf_broadcast_odometry_;    // according transformation for the tf broadcaster

    // topics to subscribe, callback is called for new messages arriving
    ros::Subscriber topic_sub_CMD_pltf_twist_;          // issued command to be achieved by the platform
    ros::Subscriber topic_sub_EM_stop_state_;           // current emergency stop state (free, active, confirmed)
    ros::Subscriber topic_sub_drive_diagnostic_;        // status of drive chain (initializing, error, normal)

    //subscribe to JointStates topic
    //ros::Subscriber topic_sub_joint_states_;
    ros::Subscriber topic_sub_joint_controller_states_;

    // diagnostic stuff
    diagnostic_updater::Updater updater_;

    // controller Timer
    ros::Timer timer_ctrl_step_;

    // member variables
    UndercarriageCtrl * ucar_ctrl_;          // instantiate undercarriage controller
    bool is_initialized_bool_;               // flag wether node is already up and running
    bool is_ucarr_geom_initialized_bool_;
    bool broadcast_tf_;                      // flag wether to broadcast the tf from odom to base_link
    int drive_chain_diagnostic_;             // flag whether base drive chain is operating normal 
    ros::Time last_time_;                    // time Stamp for last odometry measurement
    ros::Time joint_state_odom_stamp_;       // time stamp of joint states used for current odometry calc
    double sample_time_, timeout_;
    int iwatchdog_;
    double max_vel_trans_, max_vel_rot_;

    OdometryTracker* odom_tracker_;

    int m_iNumJoints;
    int m_iNumWheels;
    bool m_bEMStopActive;

    bool has_target;

    diagnostic_msgs::DiagnosticStatus diagnostic_status_lookup_; // used to access defines for warning levels

    // Constructor
    NodeClass()
    : ucar_ctrl_(0)
    {
      // initialization of variables
      is_initialized_bool_ = false;
      is_ucarr_geom_initialized_bool_ = false;
      broadcast_tf_ = true;
      iwatchdog_ = 0;
      last_time_ = ros::Time::now();
      sample_time_ = 0.020; // TODO: read from parameter server
      // set status of drive chain to WARN by default
      drive_chain_diagnostic_ = diagnostic_status_lookup_.OK; //WARN; <- THATS FOR DEBUGGING ONLY!
      has_target = false;     

      // Parameters are set within the launch file
      // read in timeout for watchdog stopping the controller.
      if (n.hasParam("timeout"))
      {
        n.getParam("timeout", timeout_);
        ROS_INFO("Timeout loaded from Parameter-Server is: %fs", timeout_);
      }
      else
      {
        ROS_WARN("No parameter timeout on Parameter-Server. Using default: 1.0s");
        timeout_ = 1.0;
      }
      if ( timeout_ < sample_time_ )
      {
        ROS_WARN("Specified timeout < sample_time. Setting timeout to sample_time = %fs", sample_time_);
        timeout_ = sample_time_;
      }

      if (n.hasParam("max_trans_velocity"))
      {
        n.getParam("max_trans_velocity", max_vel_trans_);
        ROS_INFO("Max translational velocity loaded from Parameter-Server is: %fs", max_vel_trans_);
      }
      else
      {
        ROS_WARN("No parameter max_trans_velocity on Parameter-Server. Using default: 1.1 m/s");
        max_vel_trans_ = 1.1;
      }
      if (n.hasParam("max_rot_velocity"))
      {
        n.getParam("max_rot_velocity", max_vel_rot_);
        ROS_INFO("Max rotational velocity loaded from Parameter-Server is: %fs", max_vel_rot_);
      }
      else
      {
        ROS_WARN("No parameter max_rot_velocity on Parameter-Server. Using default: 1.8 rad/s");
        max_vel_rot_ = 1.8;
      }
      if (n.hasParam("broadcast_tf"))
      {
        n.getParam("broadcast_tf", broadcast_tf_);
      }

      const std::string frame_id = n.param("frame_id", std::string("odom"));
      const std::string child_frame_id = n.param("child_frame_id", std::string("base_footprint"));
      const double cov_pose = n.param("cov_pose", 0.1);
      const double cov_twist = n.param("cov_twist", 0.1);

      odom_tracker_ = new OdometryTracker(frame_id, child_frame_id, cov_pose, cov_twist);

     // vector of wheels
     std::vector<UndercarriageCtrl::WheelParams> wps;

     // configuration of ucar_ctrl by yaml-files
     if(cob_omni_drive_controller::parseWheelParams(wps,n)){

        m_iNumWheels = wps.size();
        m_iNumJoints = m_iNumWheels * 2;

         ucar_ctrl_ = new UndercarriageCtrl(wps);
         is_ucarr_geom_initialized_bool_ = true;
     }

      // implementation of topics
      // published topics
      //topic_pub_joint_state_cmd_ = n.advertise<sensor_msgs::JointState>("joint_command", 1);
      topic_pub_controller_joint_command_ = n.advertise<control_msgs::JointTrajectoryControllerState> ("joint_command", 1);

      topic_pub_odometry_ = n.advertise<nav_msgs::Odometry>("odometry", 1);

      // subscribed topics
      topic_sub_CMD_pltf_twist_ = n.subscribe("command", 1, &NodeClass::topicCallbackTwistCmd, this);
      topic_sub_EM_stop_state_ = n.subscribe("/emergency_stop_state", 1, &NodeClass::topicCallbackEMStop, this);
      topic_sub_drive_diagnostic_ = n.subscribe("diagnostic", 1, &NodeClass::topicCallbackDiagnostic, this);



      //topic_sub_joint_states_ = n.subscribe("/joint_states", 1, &NodeClass::topicCallbackJointStates, this);
      topic_sub_joint_controller_states_ = n.subscribe("state", 1, &NodeClass::topicCallbackJointControllerStates, this);

      // diagnostics
      updater_.setHardwareID(ros::this_node::getName());
      updater_.add("initialization", this, &NodeClass::diag_init);

      //set up timer to cyclically call controller-step
      timer_ctrl_step_ = n.createTimer(ros::Duration(sample_time_), &NodeClass::timerCallbackCtrlStep, this);

      if (is_ucarr_geom_initialized_bool_)
            is_initialized_bool_ = true;
    }


    // Destructor
    ~NodeClass() 
    {
      topic_sub_CMD_pltf_twist_.shutdown();
      topic_sub_EM_stop_state_.shutdown();
      topic_sub_drive_diagnostic_.shutdown();
      topic_sub_joint_controller_states_.shutdown();

      timer_ctrl_step_.stop();
      delete ucar_ctrl_;
      delete odom_tracker_;
    }

    void diag_init(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
      if(is_initialized_bool_)
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "");
      else
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "");
      stat.add("Initialized", is_initialized_bool_);
    }

    // Listen for Pltf Cmds
    void topicCallbackTwistCmd(const geometry_msgs::Twist::ConstPtr& msg)
    {
      // create instance of pltState which is initialized with zero values
      UndercarriageCtrl::PlatformState pltState;

      if( (fabs(msg->linear.x) > max_vel_trans_) || (fabs(msg->linear.y) > max_vel_trans_) || (fabs(msg->angular.z) > max_vel_rot_)){
        if(fabs(msg->linear.x) > max_vel_trans_){
            ROS_DEBUG_STREAM("Recevied cmdVelX: " << msg->linear.x <<
                             ", which is bigger than the maximal allowed translational velocity: " <<  max_vel_trans_ << " so stop the robot");
        }
        if(fabs(msg->linear.y) > max_vel_trans_){
            ROS_DEBUG_STREAM("Recevied cmdVelY: " << msg->linear.x <<
                             ", which is bigger than the maximal allowed translational velocity: " <<  max_vel_trans_ << " so stop the robot");
        }
        if(fabs(msg->angular.z) > max_vel_rot_){
            ROS_DEBUG_STREAM("Recevied cmdVelTh: " << msg->angular.z <<
                             ", which is bigger than the maximal allowed rotational velocity: " << max_vel_rot_ << " so stop the robot");
        }

        // pltState is initialized with zero values, so it isnt necessary to set them explicitly

      }
      else{
        // setter-methods convert SI-Units (m/s) to mm/s because controller works with those
        pltState.setVelX(msg->linear.x);
        pltState.setVelY(msg->linear.y);
        pltState.dRotRobRadS = msg->angular.z;
      }

      iwatchdog_ = 0;

      // only process if controller is already initialized
      if (is_initialized_bool_ && drive_chain_diagnostic_ == diagnostic_status_lookup_.OK){
        ROS_DEBUG("received new velocity command [cmdVelX=%3.5f,cmdVelY=%3.5f,cmdVelTh=%3.5f]",
                  msg->linear.x, msg->linear.y, msg->angular.z);

        //std::cout << "PLT_VELO_NEW: " << pltState.dVelLongMMS << " , " << pltState.dVelLatMMS << " , " << pltState.dRotRobRadS << std::endl;
        // Set desired values for Plattform State to UndercarriageCtrl (setpoint setting
        has_target =  msg->linear.x!= 0 ||  msg->linear.y!=0 ||  msg->angular.z  != 0;
        ucar_ctrl_->setTarget(pltState);
      }
      else{
        //std::cout << "PLT_VELO_NEW: " << pltState.dVelLongMMS << " , " << pltState.dVelLatMMS << " , " << pltState.dRotRobRadS << std::endl;
        // Set desired values for Plattform State to zero (setpoint setting)
        // pltState will be initialized with zero values
        pltState = UndercarriageCtrl::PlatformState();
        ucar_ctrl_->setTarget(pltState);

        ROS_DEBUG("Forced platform-velocity cmds to zero");
      }
    }

    // Listen for Emergency Stop
    void topicCallbackEMStop(const cob_msgs::EmergencyStopState::ConstPtr& msg)
    {
      int current_EM_state = msg->emergency_state;

      if (current_EM_state == msg->EMFREE){
        // Reset EM flag in Ctrlr
        if (is_initialized_bool_){
            setEMStopActive(false);
            ROS_DEBUG("Undercarriage Controller EM-Stop released");
        }
      }
      else{
        ROS_DEBUG("Undercarriage Controller stopped due to EM-Stop");

        // Set desired values for Plattform State to zero (setpoint setting)
        // pltState will be initialized with zero values
        UndercarriageCtrl::PlatformState pltState;
        ucar_ctrl_->setTarget(pltState);

        ROS_DEBUG("Forced platform-velocity cmds to zero");

        // Set EM flag and stop Ctrlr
        setEMStopActive(true);
      }
    }

    // Listens for status of underlying hardware (base drive chain)
    void topicCallbackDiagnostic(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg)
    {
      control_msgs::JointTrajectoryControllerState joint_state_cmd;

      // prepare joint_cmds for heartbeat (compose header)
      joint_state_cmd.header.stamp = ros::Time::now();
      //joint_state_cmd.header.frame_id = frame_id; //Where to get this id from?
      // ToDo: configure over Config-File (number of motors) and Msg
      // assign right size to JointState data containers
      //joint_state_cmd.set_name_size(m_iNumMotors);
      joint_state_cmd.desired.positions.resize(m_iNumJoints);
      joint_state_cmd.desired.velocities.resize(m_iNumJoints);            
      //joint_state_cmd.desired.effort.resize(m_iNumJoints);

      // TODO: Generic adapation reqiuerd for joint_names when base_drive_chain will be updated
      joint_state_cmd.joint_names.push_back("fl_caster_r_wheel_joint");
      joint_state_cmd.joint_names.push_back("fl_caster_rotation_joint");
      joint_state_cmd.joint_names.push_back("bl_caster_r_wheel_joint");
      joint_state_cmd.joint_names.push_back("bl_caster_rotation_joint");
      joint_state_cmd.joint_names.push_back("br_caster_r_wheel_joint");
      joint_state_cmd.joint_names.push_back("br_caster_rotation_joint");
      joint_state_cmd.joint_names.push_back("fr_caster_r_wheel_joint");
      joint_state_cmd.joint_names.push_back("fr_caster_rotation_joint");
      joint_state_cmd.joint_names.resize(m_iNumJoints);

      // compose jointcmds
      for(int i=0; i<m_iNumJoints; i++)
      {
        joint_state_cmd.desired.positions[i] = 0.0;
        joint_state_cmd.desired.velocities[i] = 0.0;
        //joint_state_cmd.desired.effort[i] = 0.0;
      }

      // set status of underlying drive chain to member variable 
      drive_chain_diagnostic_ = msg->level;

      // if controller is already started up ...
      if (is_initialized_bool_){
        // ... but underlying drive chain is not yet operating normal
        if (drive_chain_diagnostic_ != diagnostic_status_lookup_.OK){
            has_target = false;
            // halt controller
            ROS_DEBUG("drive chain not availlable: halt Controller");

            // Set EM flag to Ctrlr (resets internal states)
            setEMStopActive(true);

            // Set desired values for Plattform State to zero (setpoint setting)
            // pltState will be initialized with zero values
            UndercarriageCtrl::PlatformState pltState;
            ucar_ctrl_->setTarget(pltState);

            ROS_DEBUG("Forced platform-velocity cmds to zero");
        }
      }
      // ... while controller is not initialized send heartbeats to keep motors alive
      else
      {
        // ... as soon as base drive chain is initialized
        if(drive_chain_diagnostic_ != diagnostic_status_lookup_.WARN)
        {
          // publish zero-vel. jointcmds to avoid Watchdogs stopping ctrlr
          topic_pub_controller_joint_command_.publish(joint_state_cmd);
        }
      }
    }

    void topicCallbackJointControllerStates(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg) {
      int num_joints = msg->joint_names.size();

      // replaces the vectors per parameter with a vector of wheelStates which combines the wheel specfic params
      std::vector<UndercarriageCtrl::WheelState> wStates;
      wStates.assign(m_iNumWheels, UndercarriageCtrl::WheelState());

      joint_state_odom_stamp_ = msg->header.stamp;

      for(int i = 0; i < num_joints; i++)
      {
        // associate inputs to according steer and drive joints
        // ToDo: specify this globally (Prms-File or config-File or via msg-def.)
        if(msg->joint_names[i] ==  "fl_caster_r_wheel_joint")
        {
             wStates[0].dVelGearDriveRadS = msg->actual.velocities[i];
        }
        if(msg->joint_names[i] ==  "bl_caster_r_wheel_joint")
        {
            wStates[1].dVelGearDriveRadS = msg->actual.velocities[i];
        }
        if(msg->joint_names[i] ==  "br_caster_r_wheel_joint")
        {
            wStates[2].dVelGearDriveRadS = msg->actual.velocities[i];
        }
        if(msg->joint_names[i] ==  "fr_caster_r_wheel_joint")
        {
            wStates[3].dVelGearDriveRadS = msg->actual.velocities[i];
        }
        if(msg->joint_names[i] ==  "fl_caster_rotation_joint")
        {
            wStates[0].dAngGearSteerRad = msg->actual.positions[i];
            wStates[0].dVelGearSteerRadS = msg->actual.velocities[i];
        }
        if(msg->joint_names[i] ==  "bl_caster_rotation_joint")
        {
            wStates[1].dAngGearSteerRad = msg->actual.positions[i];
            wStates[1].dVelGearSteerRadS = msg->actual.velocities[i];
        }
        if(msg->joint_names[i] ==  "br_caster_rotation_joint")
        {
            wStates[2].dAngGearSteerRad = msg->actual.positions[i];
            wStates[2].dVelGearSteerRadS = msg->actual.velocities[i];
        }
        if(msg->joint_names[i] ==  "fr_caster_rotation_joint")
        {
            wStates[3].dAngGearSteerRad = msg->actual.positions[i];
            wStates[3].dVelGearSteerRadS = msg->actual.velocities[i];
        }
      }

      // Set measured Wheel Velocities and Angles to Controler Class (implements inverse kinematic)
      ucar_ctrl_->updateWheelStates(wStates);

      // calculate odometry every time
      UpdateOdometry();
    }

    void timerCallbackCtrlStep(const ros::TimerEvent& e) {
      CalcCtrlStep();
    }

    // other function declarations
    // Initializes controller
    bool InitCtrl();
    // perform one control step, calculate inverse kinematics and publish updated joint cmd's (if no EMStop occurred)
    void CalcCtrlStep();
    // acquires the current undercarriage configuration from base_drive_chain
    // calculates odometry from current measurement values and publishes it via an odometry topic and the tf broadcaster
    void UpdateOdometry();

    // set EM Flag and stop ctrlr if active
    void setEMStopActive(bool bEMStopActive);

};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "undercarriage_ctrl");

  // construct nodeClass
  // automatically do initializing of controller, because it's not directly depending any hardware components
  NodeClass nodeClass;

  // verify init status
  if( nodeClass.is_initialized_bool_ ) {
    nodeClass.last_time_ = ros::Time::now();
    ROS_INFO("New Undercarriage control successfully initialized.");
  } else {
    ROS_FATAL("Undercarriage control initialization failed!");
    throw std::runtime_error("Undercarriage control initialization failed, check yaml-Files!");
  }

  /* 
     CALLBACKS being executed are:
     - actual motor values -> calculating direct kinematics and doing odometry (topicCallbackJointControllerStates)
     - timer callback -> calculate controller step at a rate of sample_time_ (timerCallbackCtrlStep)
     - other topic callbacks (diagnostics, command, em_stop_state)
     */
  ros::spin();

  return 0;
}

//##################################
//#### function implementations ####

// perform one control step, calculate inverse kinematics and publish updated joint cmd's (if no EMStop occurred)
void NodeClass::CalcCtrlStep()
{
  // WheelStates will be initialized with zero-values
  std::vector<UndercarriageCtrl::WheelState> wStates;
  wStates.assign(m_iNumWheels, UndercarriageCtrl::WheelState());

  // create control_msg
  control_msgs::JointTrajectoryControllerState joint_state_cmd;

  int j = 0, k = 0;
  iwatchdog_ += 1;

  // if controller is initialized and underlying hardware is operating normal
  if (is_initialized_bool_) //  && (drive_chain_diagnostic_ != diagnostic_status_lookup_.OK))
  {
    // as soon as (but only as soon as) platform drive chain is initialized start to send velocity commands
    // Note: topicCallbackDiagnostic checks whether drives are operating nominal.
    // -> if warning or errors are issued target velocity is set to zero

    // perform one control step,
    // get the resulting cmd's for the wheel velocities and -angles from the controller class
    // and output the achievable pltf velocity-cmds (if velocity limits where exceeded)
    ucar_ctrl_->calcControlStep(wStates, sample_time_, false);

    // if drives not operating nominal -> force commands to zero
    if(drive_chain_diagnostic_ != diagnostic_status_lookup_.OK){
        for(int i = 0; i < wStates.size(); i++){
            wStates[i].dAngGearSteerRad = 0.0;
            wStates[i].dVelGearSteerRadS = 0.0;
        }
    }

    // compose jointcmds of control_msg

    // compose header of control_msg
    joint_state_cmd.header.stamp = ros::Time::now();

    //Where to get this id from?
    //joint_state_cmd.header.frame_id = frame_id;

    // ToDo: configure over Config-File (number of motors) and Msg
    // assign right size to JointState data containers
    //joint_state_cmd.set_name_size(m_iNumJoints);

    joint_state_cmd.desired.positions.resize(m_iNumJoints);
    joint_state_cmd.desired.velocities.resize(m_iNumJoints);            
    //joint_state_cmd.effort.resize(m_iNumJoints);
    joint_state_cmd.joint_names.push_back("fl_caster_r_wheel_joint");
    joint_state_cmd.joint_names.push_back("fl_caster_rotation_joint");
    joint_state_cmd.joint_names.push_back("bl_caster_r_wheel_joint");
    joint_state_cmd.joint_names.push_back("bl_caster_rotation_joint");
    joint_state_cmd.joint_names.push_back("br_caster_r_wheel_joint");
    joint_state_cmd.joint_names.push_back("br_caster_rotation_joint");
    joint_state_cmd.joint_names.push_back("fr_caster_r_wheel_joint");
    joint_state_cmd.joint_names.push_back("fr_caster_rotation_joint");
    joint_state_cmd.joint_names.resize(m_iNumJoints);

    // compose data body of control_msg
    for(int i = 0; i<m_iNumJoints; i++)
    {
      if(iwatchdog_ < (int) std::floor(timeout_/sample_time_) && has_target)
      {
        // for steering motors
        if( i == 1 || i == 3 || i == 5 || i == 7) // ToDo: specify this via the Msg
        {
            joint_state_cmd.desired.positions[i] = wStates[j].dAngGearSteerRad;
            joint_state_cmd.desired.velocities[i] = wStates[j].dVelGearSteerRadS;
            j = j + 1;
        }
        else
        {
          joint_state_cmd.desired.positions[i] = 0.0;
          joint_state_cmd.desired.velocities[i] = wStates[k].dVelGearDriveRadS;
          k = k + 1;
        }
      }
      else
      {
        joint_state_cmd.desired.positions[i] = 0.0;
        joint_state_cmd.desired.velocities[i] = 0.0;
      }
    }

    // publish jointcmds
    topic_pub_controller_joint_command_.publish(joint_state_cmd);
  }

}

// calculates odometry from current measurement values
// and publishes it via an odometry topic and the tf broadcaster
void NodeClass::UpdateOdometry()
{
  // if drive chain already initialized process joint data
  //if (drive_chain_diagnostic_ != diagnostic_status_lookup_.OK)
  UndercarriageCtrl::PlatformState pltState;
  if (is_initialized_bool_)
  {

    // Get resulting Pltf Velocities from Ctrl-Class (result of forward kinematics)
    // !Careful! Controller internally calculates with mm instead of m
    // ToDo: change internal calculation to SI-Units
    ucar_ctrl_->calcDirect(pltState);
  }
  odom_tracker_->track(joint_state_odom_stamp_, (joint_state_odom_stamp_-last_time_).toSec(), pltState.getVelX(), pltState.getVelY(), pltState.dRotRobRadS);
  last_time_ = joint_state_odom_stamp_;

  nav_msgs::Odometry odom_top = odom_tracker_->getOdometry();

  if (broadcast_tf_ == true)
  {
    // compose and publish transform for tf package
    geometry_msgs::TransformStamped odom_tf;
    // compose header
    odom_tf.header.stamp = odom_top.header.stamp;
    odom_tf.header.frame_id = "/odom_combined";
    odom_tf.child_frame_id = "/base_footprint";
    // compose data container
    odom_tf.transform.translation.x = odom_top.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_top.pose.pose.position.y;
    odom_tf.transform.rotation = odom_top.pose.pose.orientation;

    // publish the transform (for debugging, conflicts with robot-pose-ekf)
    tf_broadcast_odometry_.sendTransform(odom_tf);
  }

  // publish odometry msg
  topic_pub_odometry_.publish(odom_top);
}

// set EM Flag and stop ctrlr if active
void NodeClass::setEMStopActive(bool bEMStopActive)
{
    m_bEMStopActive = bEMStopActive;

    std::vector<UndercarriageCtrl::WheelState> wStates;
    wStates.assign(m_iNumWheels, UndercarriageCtrl::WheelState());

    // if emergency stop reset ctrlr to zero
    if(m_bEMStopActive)
    {
//        std::cout << "EMStop: " << "Active" << std::endl;
        has_target = false;
        // reset and update current wheel states but keep current dAngGearSteerRad per wheelState
        ucar_ctrl_->calcControlStep(wStates, sample_time_, true);

        // set current wheel states with previous reset and updated wheelStates
        ucar_ctrl_->updateWheelStates(wStates);
    }
//    else
//        std::cout << "EMStop: " << "Inactive" << std::endl;
}



