/**
 * @file /src/velocity_smoother_nodelet.cpp
 *
 * @brief Velocity smoother implementation.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/yujin_ocs/hydro/yocs_velocity_smoother/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <cob_base_velocity_smoother/velocity_smoother.h>

#include <boost/thread.hpp>

/*****************************************************************************
 ** Preprocessing
 *****************************************************************************/

#define PERIOD_RECORD_SIZE    5
#define ZERO_VEL_COMMAND      geometry_msgs::Twist();
#define IS_ZERO_VEOCITY(a)   ((a.linear.x == 0.0) && (a.linear.y == 0.0) && (a.angular.z == 0.0))

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cob_base_velocity_smoother {

/*********************
** Implementation
**********************/

VelocitySmoother::VelocitySmoother(const std::string &name)
: name(name)
, input_active(false)
, pr_next(0)
, dynamic_reconfigure_server(NULL)
{
}

void VelocitySmoother::reconfigCB(cob_base_velocity_smoother::paramsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : \n\tSpeedLimit: %f %f %f \n\tAccLimit: %f %f %f\n\tDecelFactor: %f, SafeFactor: %f \n\tRobotFeedback: %d",
           config.speed_lim_vx, config.speed_lim_vy, config.speed_lim_w, config.accel_lim_vx, config.accel_lim_vy, config.accel_lim_w, config.decel_factor, config.safe_factor, config.robot_feedback);

  robot_feedback = (VelocitySmoother::RobotFeedbackType) config.robot_feedback;
  speed_lim_vx  = config.speed_lim_vx;
  speed_lim_vy  = config.speed_lim_vy;
  speed_lim_w   = config.speed_lim_w;
  accel_lim_vx  = config.accel_lim_vx;
  accel_lim_vy  = config.accel_lim_vy;
  accel_lim_w   = config.accel_lim_w;
  decel_factor  = config.decel_factor;
  safe_factor   = config.safe_factor;
}

void VelocitySmoother::velocityCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(_mutex);

  ros::Time now = ros::Time::now();
  double duration = (now - last_cb_time).toSec();
  
  if (period_record.size() < PERIOD_RECORD_SIZE)
  {
    period_record.push_back(duration);
  }
  else
  {
    period_record[pr_next] = duration;
  }

  pr_next++;
  pr_next %= period_record.size();
  last_cb_time = now;

  if (period_record.size() <= PERIOD_RECORD_SIZE/2)
  {
    // wait until we have some values; make a reasonable assumption meanwhile
    cb_avg_time = 1.0/frequency;
  }
  else
  {
    // enough; recalculate with the latest input
    cb_avg_time = median(period_record);
  }

  // Bound speed with the maximum values
  target_vel.linear.x  =
      msg->linear.x  > 0.0 ? std::min(msg->linear.x,  speed_lim_vx) : std::max(msg->linear.x,  -speed_lim_vx);
  target_vel.linear.y  =
      msg->linear.y  > 0.0 ? std::min(msg->linear.y,  speed_lim_vy) : std::max(msg->linear.y,  -speed_lim_vy);
  target_vel.angular.z =
      msg->angular.z > 0.0 ? std::min(msg->angular.z, speed_lim_w)  : std::max(msg->angular.z, -speed_lim_w);

  input_active = true;
}

void VelocitySmoother::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(_mutex);

  if (robot_feedback == ODOMETRY)
    current_vel = msg->twist.twist;

  // ignore otherwise
}

void VelocitySmoother::robotVelCB(const geometry_msgs::Twist::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(_mutex);

  if (robot_feedback == COMMANDS)
    current_vel = *msg;

  // ignore otherwise
}

void VelocitySmoother::spin(const ros::TimerEvent& event)
{
  boost::mutex::scoped_lock lock(_mutex);

  //double period = 1.0/frequency;
  double period = (event.current_real - event.last_real).toSec();
  double last_command = (event.current_real - last_cb_time).toSec();

  if ((input_active == true) && (cb_avg_time > 0.0) &&
      (last_command > std::min(PERIOD_RECORD_SIZE*cb_avg_time, 0.5))) //missed PERIOD_RECORD_SIZE input cycles or no input for 0.5 secs
  {
    // Velocity input not active anymore; normally last command is a zero-velocity one, but reassure
    // this, just in case something went wrong with our input, or he just forgot good manners...
    input_active = false;
    ROS_WARN_STREAM("Velocity Smoother : input got inactive");
    if (IS_ZERO_VEOCITY(target_vel) == false)
    {
      ROS_WARN_STREAM("...leaving us a non-zero target velocity ("
            << target_vel.linear.x << ", " << target_vel.linear.y << ", " << target_vel.angular.z << "), zeroing...[" << name << "]");
      target_vel = ZERO_VEL_COMMAND;
    }
  }

  double accel_vx = (input_active)?accel_lim_vx:safe_factor*accel_lim_vx;
  double accel_vy = (input_active)?accel_lim_vy:safe_factor*accel_lim_vy;
  double accel_w  = (input_active)?accel_lim_w :safe_factor*accel_lim_w;

  double decel_vx = decel_factor*accel_vx;
  double decel_vy = decel_factor*accel_vy;
  double decel_w  = decel_factor*accel_w;

  if ((robot_feedback != NONE) && (input_active == true) && (cb_avg_time > 0.0) &&
      ((last_command > std::min(PERIOD_RECORD_SIZE*cb_avg_time, 0.5))    || //missed PERIOD_RECORD_SIZE input cycles or no input for 0.5 secs
       (std::abs(current_vel.linear.x  - last_cmd_vel.linear.x)  > 0.2)  ||
       (std::abs(current_vel.linear.y  - last_cmd_vel.linear.y)  > 0.2)  ||
       (std::abs(current_vel.angular.z - last_cmd_vel.angular.z) > 2.0)))
  {
    // If the publisher has been inactive for a while, or if our current commanding differs a lot
    // from robot velocity feedback, we cannot trust the former; rely on robot's feedback instead
    // This can happen mainly due to preemption of current controller on velocity multiplexer.
    // TODO: current command/feedback difference thresholds are arbitrary; they should somehow
    // be proportional to max v and w...
    ROS_WARN("Last command differs to much from robot velocity feedback (%s). Use feedback instead!",
              robot_feedback == ODOMETRY ? "ODOMETRY" : "COMMAND");
    last_cmd_vel = current_vel;
  }

  if ((target_vel.linear.x  != last_cmd_vel.linear.x) ||
      (target_vel.linear.y  != last_cmd_vel.linear.y) ||
      (target_vel.angular.z != last_cmd_vel.angular.z))
  {
    // Try to reach target velocity ensuring that we don't exceed the acceleration limits
    geometry_msgs::Twist cmd_vel = target_vel;

    double vx_inc, vy_inc, w_inc;
    double ax, ay, aw;
    double max_ax, max_ay, max_aw;
    double actual_ax, actual_ay, actual_aw;

    vx_inc = target_vel.linear.x - last_cmd_vel.linear.x;
    ax = vx_inc/cb_avg_time;
    max_ax = (ax > 0.0)?accel_vx:decel_vx;
    actual_ax = (std::abs(ax) > max_ax)?sign(ax)*max_ax:ax;

    vy_inc = target_vel.linear.y - last_cmd_vel.linear.y;
    ay = vy_inc/cb_avg_time;
    max_ay = (ay > 0.0)?accel_vy:decel_vy;
    actual_ay = (std::abs(ay) > max_ay)?sign(ay)*max_ay:ay;

    w_inc = target_vel.angular.z - last_cmd_vel.angular.z;
    aw = w_inc/cb_avg_time;
    max_aw = (aw > 0.0)?accel_w:decel_w;
    actual_aw = (std::abs(aw) > max_aw)?sign(aw)*max_aw:aw;

    cmd_vel.linear.x  = last_cmd_vel.linear.x  + actual_ax*period;
    cmd_vel.linear.y  = last_cmd_vel.linear.y  + actual_ay*period;
    cmd_vel.angular.z = last_cmd_vel.angular.z + actual_aw*period;

    smooth_vel_pub.publish(cmd_vel);
    last_cmd_vel = cmd_vel;
  }
  else if (input_active == true)
  {
    smooth_vel_pub.publish(target_vel);
  }
}

/**
 * Initialise from private NodeHandle.
 * @param nh : private NodeHandle
 * @return bool : success or failure
 */
bool VelocitySmoother::init(ros::NodeHandle& nh)
{
  // Dynamic Reconfigure
  dynamic_reconfigure_callback = boost::bind(&VelocitySmoother::reconfigCB, this, _1, _2);

  dynamic_reconfigure_server = new dynamic_reconfigure::Server<cob_base_velocity_smoother::paramsConfig>(nh);
  dynamic_reconfigure_server->setCallback(dynamic_reconfigure_callback);

  nh.param("frequency",      frequency,     50.0);

  // Publishers and subscribers
  odometry_sub    = nh.subscribe("odometry",      1, &VelocitySmoother::odometryCB, this);
  current_vel_sub = nh.subscribe("robot_cmd_vel", 1, &VelocitySmoother::robotVelCB, this);
  raw_in_vel_sub  = nh.subscribe("raw_cmd_vel",   1, &VelocitySmoother::velocityCB, this);
  smooth_vel_pub  = nh.advertise <geometry_msgs::Twist> ("smooth_cmd_vel", 1);

  timer = nh.createTimer(ros::Duration(1.0/frequency), &VelocitySmoother::spin, this);

  return true;
}
}
