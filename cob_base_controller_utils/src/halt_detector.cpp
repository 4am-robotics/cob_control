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


#include <ros/ros.h>
#include <cob_base_controller_utils/WheelCommands.h>
#include <controller_manager_msgs/SwitchController.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

ros::ServiceClient g_halt_client;
ros::ServiceClient g_recover_client;
ros::ServiceClient g_switch_controller_client;
std::vector<std::string> g_controller_spawn;
ros::Duration g_stop_timeout;
ros::Duration g_stuck_timeout;
ros::Duration g_recover_after_stuck_timeout;
std::vector<ros::Time> g_last_wheel_commands_ok;
ros::Time g_last_diagnostic_recover_timestamp;
ros::Timer g_stop_timer;
ros::Timer g_recover_after_stuck_timer;
double g_stuck_threshold;
double g_stop_threshold;
bool g_recovering;
bool g_is_stopped;
bool g_is_stuck;
bool g_stuck_possible;
std::vector<std::string> g_diagnostic_recovery_trigger;



/**
 * Reload the controllers.
 */
void switch_controllers() {
  controller_manager_msgs::SwitchController switch_srv;
  switch_srv.request.start_controllers = g_controller_spawn;
  switch_srv.request.strictness = 1;

  if (g_switch_controller_client.call(switch_srv)) {
    if (switch_srv.response.ok) {
      ROS_INFO("Successfully switched controllers");
    } else {
      ROS_ERROR("Could not switch controllers");
    }
  } else {
    ROS_ERROR("ServiceCall failed: switch_controller");
  }
}


/**
 * Recover the base.
 */
void recover() {
  if (!g_recovering) {
    g_recovering = true;
    std_srvs::Trigger srv;
    g_recover_client.call(srv);
    switch_controllers();
    g_is_stopped = false;
    g_is_stuck = false;
    g_recovering = false;
  }
}


/**
 * Halt the base.
 */
void halt() {
  std_srvs::Trigger srv;
  g_halt_client.call(srv);
}


/**
 * Callback for the stop timer.
 */
void haltAfterStopTimeoutCallback(const ros::TimerEvent&) {
  g_is_stopped = true;
  ROS_INFO_STREAM("Halt, no movement commands received for " << g_stop_timeout << " seconds");
  halt();
}


/**
 * Callback for the recover timer after stuck.
 */
void recoverAfterStuckCallback(const ros::TimerEvent&) {
  ROS_INFO("Recovering after stuck");
  recover();
  g_stop_timer.start();

  // Wait until the next stuck is possible to prevent "stuck loop"
  g_stuck_timeout.sleep();
  g_stuck_possible = true;
}

/**
 * Checks if a substring is present in a string.
 *
 * @param string String which is checked.
 * @param substring String which is searched.
 * @returns Tue if substring is present, false otherwise.
 */
bool containsSubstring(std::string string, std::string substring) {
  if (std::strstr(string.c_str(), substring.c_str())) {
    return true;
  } else {
    return false;
  }
}

/**
 * Callback for diagnostic aggregation.
 *
 * Recovers on diagnostic errors if currently not stopped or stuck.
 *
 * @param msg DiagnosticArray message.
 */
void diagnosticsAggCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
  for (size_t i = 0; i < msg->status.size(); i++) {

    // Get status with name containing 'base' and 'wheel' or 'base' and 'rotation'
    if (msg->status[i].level > diagnostic_msgs::DiagnosticStatus::WARN &&
        containsSubstring(msg->status[i].name, "base") &&
        (containsSubstring(msg->status[i].name, "wheel") ||
         containsSubstring(msg->status[i].name, "rotation"))) {

      for (size_t j = 0; j < msg->status[i].values.size(); j++) {

        // Check if error value is in trigger list and recover eventually
        if (msg->status[i].values[j].key == "errors") {
          bool error = false;
          for (int k = 0; k < g_diagnostic_recovery_trigger.size(); k++) {
            if (containsSubstring(msg->status[i].values[j].value, g_diagnostic_recovery_trigger[k])) {
              error = true;
            }
          }
          if (error && !g_is_stuck && !g_is_stopped && ros::Time::now() - g_last_diagnostic_recover_timestamp >= ros::Duration(2.0)) {
            ROS_INFO("Recovering from diagnostic error");
            recover();
            g_last_diagnostic_recover_timestamp = ros::Time::now();
          }
        }
      }
    }
  }
}

/**
 * Callback for wheel commands.
 *
 * Calls a halt if the error of the wheel commands exceeded set threshold for
 * set timeout.
 *
 * @param msg WheelCommands message.
 */
void wheelCommandsCallback(const cob_base_controller_utils::WheelCommands::ConstPtr& msg) {

  // Initialize last timestamp
  if (g_last_wheel_commands_ok.size() == 0) {
    g_last_wheel_commands_ok.resize(msg->steer_target_error.size(), msg->header.stamp);
  }

  // Check if a wheel exceeds threshold
  for (size_t i = 0; i < msg->steer_target_error.size(); ++i) {
    if (fabs(msg->steer_target_error[i]) < g_stuck_threshold) {
      g_last_wheel_commands_ok[i] = msg->header.stamp;
    }
    else {
      ROS_DEBUG_STREAM("Wheel " << i << " exceeded stuck threshold");
    }
  }

  // Check if a wheel exceeds timeout
  bool exceeded = false;
  for (size_t i = 0; i < g_last_wheel_commands_ok.size(); ++i) {
    if ((msg->header.stamp - g_last_wheel_commands_ok[i]) >= g_stuck_timeout) {
      exceeded = true;
      ROS_ERROR_STREAM("Wheel " << i << " exceeded stuck threshold and timeout");
    }
  }

  // Halt if exceeded and allowed
  if (exceeded && !g_is_stuck && !g_is_stopped && g_stuck_possible) {
    g_is_stuck = true;
    g_stuck_possible = false;
    g_recover_after_stuck_timer.stop();
    ROS_ERROR("Halt, robot is stuck");
    halt();
    g_recover_after_stuck_timer.start();
  }
}


/**
 * Callback for base command twists.
 *
 * Restarts the stop_timer if twist exceeds the threshold. Additionally recovers
 * the base if a twist exceeds the threshold, the base ist not stuck and not yet
 * recovered.
 *
 * @param msg Twist message.
 */
void baseCommandsCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  if (fabs(msg->linear.x) >= g_stop_threshold ||
       fabs(msg->linear.y) >= g_stop_threshold ||
       fabs(msg->linear.z) >= g_stop_threshold ||
       fabs(msg->angular.x) >= g_stop_threshold ||
       fabs(msg->angular.y) >= g_stop_threshold ||
       fabs(msg->angular.z) >= g_stop_threshold) {
    g_stop_timer.stop();
    if (!g_is_stuck) {
      if (g_is_stopped) {
        ROS_INFO("Recovering after stopped");
        recover();
      }
      ROS_DEBUG("Robot is moving, restarting halt timer");
      g_stop_timer.start();
    }
  }
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cob_halt_detector");

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Get ROS parameter
  if (!nh_priv.getParam("stop_threshold", g_stop_threshold)) {
    ROS_ERROR("Please provide a stop threshold");
    return 1;
  }

  if (!nh_priv.getParam("stuck_threshold", g_stuck_threshold)) {
    ROS_ERROR("Please provide a stuck threshold");
    return 1;
  }

  double timeout;
  if (!nh_priv.getParam("stop_timeout", timeout) || timeout <= 0.0) {
    ROS_ERROR("Please provide valid stop timeout");
    return 1;
  }
  g_stop_timeout = ros::Duration(timeout);

  if (!nh_priv.getParam("stuck_timeout", timeout) || timeout <= 0.0) {
    ROS_ERROR("Please provide valid stuck timeout");
    return 1;
  }
  g_stuck_timeout = ros::Duration(timeout);

  if (!nh_priv.getParam("recover_after_stuck_timeout", timeout) || timeout <= 0.0) {
    ROS_ERROR("Please provide valid timeout for recovering after stuck");
    return 1;
  }
  g_recover_after_stuck_timeout = ros::Duration(timeout);

  ros::Subscriber diagnostics_agg_sub;
  if (nh_priv.hasParam("diagnostic_recovery_trigger")) {
    nh_priv.getParam("diagnostic_recovery_trigger", g_diagnostic_recovery_trigger);
    if (g_diagnostic_recovery_trigger.size() > 0) {
      diagnostics_agg_sub = nh.subscribe("/diagnostics_agg", 10, diagnosticsAggCallback);
    }
  }
  g_last_diagnostic_recover_timestamp = ros::Time(0);

  g_controller_spawn = {"joint_state_controller", "twist_controller"};

  g_last_wheel_commands_ok = {};
  g_recovering = false;
  g_is_stopped = false;
  g_is_stuck = false;
  g_stuck_possible = true;

  g_halt_client = nh.serviceClient<std_srvs::Trigger>("driver/halt");
  g_recover_client = nh.serviceClient<std_srvs::Trigger>("driver/recover");
  g_switch_controller_client = nh.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/switch_controller");
  g_halt_client.waitForExistence();
  g_recover_client.waitForExistence();
  g_switch_controller_client.waitForExistence();

  g_stop_timer = nh.createTimer(g_stop_timeout, haltAfterStopTimeoutCallback, true, false); // oneshot=true, auto_start=false
  g_recover_after_stuck_timer = nh.createTimer(g_recover_after_stuck_timeout, recoverAfterStuckCallback, true, false); // oneshot=true, auto_start=false

  ros::Subscriber base_commands_sub = nh.subscribe("twist_controller/command", 10, baseCommandsCallback);
  ros::Subscriber wheel_commands_sub = nh.subscribe("twist_controller/wheel_commands", 10, wheelCommandsCallback);

  ros::spin();
  return 0;
}
