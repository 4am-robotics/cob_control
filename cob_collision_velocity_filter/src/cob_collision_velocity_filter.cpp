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


#include <cob_collision_velocity_filter.h>

#include <visualization_msgs/Marker.h>

// Constructor
CollisionVelocityFilter::CollisionVelocityFilter(costmap_2d::Costmap2DROS * costmap)
{
  // create node handle
  nh_ = ros::NodeHandle("");
  pnh_ = ros::NodeHandle("~");

  m_mutex = PTHREAD_MUTEX_INITIALIZER;

  anti_collision_costmap_ = costmap;

  pnh_.param("costmap_obstacle_treshold", costmap_obstacle_treshold_, 250);

  // implementation of topics to publish (command for base and list of relevant obstacles)
  topic_pub_command_ = nh_.advertise<geometry_msgs::Twist>("command", 1);
  topic_pub_relevant_obstacles_ = nh_.advertise<nav_msgs::OccupancyGrid>("relevant_obstacles_grid", 1);

  // subscribe to twist-movement of teleop
  joystick_velocity_sub_ = nh_.subscribe<geometry_msgs::Twist>("command_in", 10,
                                                               &CollisionVelocityFilter::joystickVelocityCB, this);

  // create Timer and call getFootprint Service periodically
  double footprint_update_frequency;
  if (!pnh_.hasParam("footprint_update_frequency"))
    ROS_WARN("Used default parameter for footprint_update_frequency [1.0 Hz].");
  pnh_.param("footprint_update_frequency", footprint_update_frequency, 1.0);
  get_footprint_timer_ = pnh_.createTimer(ros::Duration(1 / footprint_update_frequency),
                                         &CollisionVelocityFilter::getFootprint, this);
  // read parameters from parameter server
  // parameters from costmap
  if (!pnh_.hasParam("global_frame"))
    ROS_WARN("Used default parameter for global_frame [/base_link]");
  pnh_.param("global_frame", global_frame_, std::string("/base_link"));

  if (!pnh_.hasParam("robot_base_frame"))
    ROS_WARN("Used default parameter for robot_frame [/base_link]");
  pnh_.param("robot_base_frame", robot_frame_, std::string("/base_link"));

  if (!pnh_.hasParam("influence_radius"))
    ROS_WARN("Used default parameter for influence_radius [1.5 m]");
  pnh_.param("influence_radius", influence_radius_, 1.5);
  closest_obstacle_dist_ = influence_radius_;
  closest_obstacle_angle_ = 0.0;

  // parameters for obstacle avoidance and velocity adjustment
  if (!pnh_.hasParam("stop_threshold"))
    ROS_WARN("Used default parameter for stop_threshold [0.1 m]");
  pnh_.param("stop_threshold", stop_threshold_, 0.10);

  if (!nh_.hasParam("obstacle_damping_dist"))
    ROS_WARN("Used default parameter for obstacle_damping_dist [5.0 m]");
  pnh_.param("obstacle_damping_dist", obstacle_damping_dist_, 5.0);
  if (obstacle_damping_dist_ <= stop_threshold_)
  {
    obstacle_damping_dist_ = stop_threshold_ + 0.01; // set to stop_threshold_+0.01 to avoid divide by zero error
    ROS_WARN("obstacle_damping_dist <= stop_threshold -> robot will stop without deceleration!");
  }

  if (!pnh_.hasParam("use_circumscribed_threshold"))
    ROS_WARN("Used default parameter for use_circumscribed_threshold_ [0.2 rad/s]");
  pnh_.param("use_circumscribed_threshold", use_circumscribed_threshold_, 0.20);

  if (!pnh_.hasParam("pot_ctrl_vmax"))
    ROS_WARN("Used default parameter for pot_ctrl_vmax [0.6]");
  pnh_.param("pot_ctrl_vmax", v_max_, 0.6);

  if (!pnh_.hasParam("pot_ctrl_vtheta_max"))
    ROS_WARN("Used default parameter for pot_ctrl_vtheta_max [0.8]");
  pnh_.param("pot_ctrl_vtheta_max", vtheta_max_, 0.8);

  if (!pnh_.hasParam("pot_ctrl_kv"))
    ROS_WARN("Used default parameter for pot_ctrl_kv [1.0]");
  pnh_.param("pot_ctrl_kv", kv_, 1.0);

  if (!pnh_.hasParam("pot_ctrl_kp"))
    ROS_WARN("Used default parameter for pot_ctrl_kp [2.0]");
  pnh_.param("pot_ctrl_kp", kp_, 2.0);

  if (!pnh_.hasParam("pot_ctrl_virt_mass"))
    ROS_WARN("Used default parameter for pot_ctrl_virt_mass [0.8]");
  pnh_.param("pot_ctrl_virt_mass", virt_mass_, 0.8);

  robot_footprint_ = anti_collision_costmap_->getRobotFootprint();

  if (robot_footprint_.size() > 4)
    ROS_WARN(
        "You have set more than 4 points as robot_footprint, cob_collision_velocity_filter can deal only with rectangular footprints so far!");

  // try to get the max_acceleration values from the parameter server
  if (!pnh_.hasParam("max_acceleration"))
    ROS_WARN("Used default parameter for max_acceleration [0.5, 0.5, 0.7]");
  XmlRpc::XmlRpcValue max_acc;
  if (pnh_.getParam("max_acceleration", max_acc))
  {
    ROS_ASSERT(max_acc.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ax_max_ = (double)max_acc[0];
    ay_max_ = (double)max_acc[1];
    atheta_max_ = (double)max_acc[2];
  }
  else
  {
    ax_max_ = 0.5;
    ay_max_ = 0.5;
    atheta_max_ = 0.7;
  }

  last_time_ = ros::Time::now().toSec();
  vx_last_ = 0.0;
  vy_last_ = 0.0;
  vtheta_last_ = 0.0;

  // dynamic reconfigure
  dynCB_ = boost::bind(&CollisionVelocityFilter::dynamicReconfigureCB, this, _1, _2);
  dyn_server_.setCallback(dynCB_);
  ROS_DEBUG("[cob_collision_velocity_filter] Initialized");
}

// Destructor
CollisionVelocityFilter::~CollisionVelocityFilter()
{
//  costmap_thread_->interrupt();
//  costmap_thread_->join();
}

// joystick_velocityCB reads twist command from joystick
void CollisionVelocityFilter::joystickVelocityCB(const geometry_msgs::Twist::ConstPtr &twist)
{
  //std::cout << "received command" << std::endl;
  ROS_DEBUG_NAMED("joystickVelocityCB", "[cob_collision_velocity_filter] Received command");
  pthread_mutex_lock(&m_mutex);

  robot_twist_linear_ = twist->linear;
  robot_twist_angular_ = twist->angular;

  pthread_mutex_unlock(&m_mutex);

  // check for relevant obstacles
  obstacleHandler();
  // stop if we are about to run in an obstacle
  performControllerStep();

}

// timer callback for periodically checking footprint
void CollisionVelocityFilter::getFootprint(const ros::TimerEvent& event)
{
  ROS_DEBUG("[cob_collision_velocity_filter] Update footprint");
  // adjust footprint
  std::vector<geometry_msgs::Point> footprint;
  footprint = anti_collision_costmap_->getRobotFootprint();

  pthread_mutex_lock(&m_mutex);

  footprint_front_ = footprint_front_initial_;
  footprint_rear_ = footprint_rear_initial_;
  footprint_left_ = footprint_left_initial_;
  footprint_right_ = footprint_right_initial_;

  robot_footprint_ = footprint;
  for (unsigned int i = 0; i < footprint.size(); i++)
  {
    if (footprint[i].x > footprint_front_)
      footprint_front_ = footprint[i].x;
    if (footprint[i].x < footprint_rear_)
      footprint_rear_ = footprint[i].x;
    if (footprint[i].y > footprint_left_)
      footprint_left_ = footprint[i].y;
    if (footprint[i].y < footprint_right_)
      footprint_right_ = footprint[i].y;
  }

  pthread_mutex_unlock(&m_mutex);

}

void CollisionVelocityFilter::dynamicReconfigureCB(
    const cob_collision_velocity_filter::CollisionVelocityFilterConfig &config, const uint32_t level)
{
  pthread_mutex_lock(&m_mutex);

  stop_threshold_ = config.stop_threshold;
  obstacle_damping_dist_ = config.obstacle_damping_dist;
  if (obstacle_damping_dist_ <= stop_threshold_)
  {
    obstacle_damping_dist_ = stop_threshold_ + 0.01; // set to stop_threshold_+0.01 to avoid divide by zero error
    ROS_WARN("obstacle_damping_dist <= stop_threshold -> robot will stop without deceleration!");
  }

  if (obstacle_damping_dist_ > config.influence_radius || stop_threshold_ > config.influence_radius)
  {
    ROS_WARN("Not changing influence_radius since obstacle_damping_dist and/or stop_threshold is bigger!");
  }
  else
  {
    influence_radius_ = config.influence_radius;
  }

  if (stop_threshold_ <= 0.0 || influence_radius_ <= 0.0)
    ROS_WARN("Turned off obstacle avoidance!");
  pthread_mutex_unlock(&m_mutex);
}

// sets corrected velocity of joystick command
void CollisionVelocityFilter::performControllerStep()
{

  double dt;
  double vx_max, vy_max;
  geometry_msgs::Twist cmd_vel, cmd_vel_in;

  cmd_vel_in.linear = robot_twist_linear_;
  cmd_vel_in.angular = robot_twist_angular_;

  cmd_vel.linear = robot_twist_linear_;
  cmd_vel.angular = robot_twist_angular_;
  dt = ros::Time::now().toSec() - last_time_;
  last_time_ = ros::Time::now().toSec();

  double vel_angle = atan2(cmd_vel.linear.y, cmd_vel.linear.x);
  vx_max = v_max_ * fabs(cos(vel_angle));
  if (vx_max > fabs(cmd_vel.linear.x))
    vx_max = fabs(cmd_vel.linear.x);
  vy_max = v_max_ * fabs(sin(vel_angle));
  if (vy_max > fabs(cmd_vel.linear.y))
    vy_max = fabs(cmd_vel.linear.y);

  //Slow down in any way while approximating an obstacle:
  if (closest_obstacle_dist_ < influence_radius_)
  {
    double F_x, F_y;
    double vx_d, vy_d, vx_factor, vy_factor;
    double kv_obst = kv_, vx_max_obst = vx_max, vy_max_obst = vy_max;

    //implementation for linear decrease of v_max:
    double obstacle_linear_slope_x = vx_max / (obstacle_damping_dist_ - stop_threshold_);
    vx_max_obst = (closest_obstacle_dist_ - stop_threshold_ + stop_threshold_ / 10.0f) * obstacle_linear_slope_x;
    if (vx_max_obst > vx_max)
      vx_max_obst = vx_max;
    else if (vx_max_obst < 0.0f)
      vx_max_obst = 0.0f;

    double obstacle_linear_slope_y = vy_max / (obstacle_damping_dist_ - stop_threshold_);
    vy_max_obst = (closest_obstacle_dist_ - stop_threshold_ + stop_threshold_ / 10.0f) * obstacle_linear_slope_y;
    if (vy_max_obst > vy_max)
      vy_max_obst = vy_max;
    else if (vy_max_obst < 0.0f)
      vy_max_obst = 0.0f;

    //Translational movement
    //calculation of v factor to limit maxspeed
    double closest_obstacle_dist_x = closest_obstacle_dist_ * cos(closest_obstacle_angle_);
    double closest_obstacle_dist_y = closest_obstacle_dist_ * sin(closest_obstacle_angle_);
    vx_d = kp_ / kv_obst * closest_obstacle_dist_x;
    vy_d = kp_ / kv_obst * closest_obstacle_dist_y;
    vx_factor = vx_max_obst / sqrt(vy_d * vy_d + vx_d * vx_d);
    vy_factor = vy_max_obst / sqrt(vy_d * vy_d + vx_d * vx_d);
    if (vx_factor > 1.0)
      vx_factor = 1.0;
    if (vy_factor > 1.0)
      vy_factor = 1.0;

    F_x = -kv_obst * vx_last_ + vx_factor * kp_ * closest_obstacle_dist_x;
    F_y = -kv_obst * vy_last_ + vy_factor * kp_ * closest_obstacle_dist_y;

    cmd_vel.linear.x = vx_last_ + F_x / virt_mass_ * dt;
    cmd_vel.linear.y = vy_last_ + F_y / virt_mass_ * dt;

  }

  // make sure, the computed and commanded velocities are not greater than the specified max velocities
  if (fabs(cmd_vel.linear.x) > vx_max)
    cmd_vel.linear.x = sign(cmd_vel.linear.x) * vx_max;
  if (fabs(cmd_vel.linear.y) > vy_max)
    cmd_vel.linear.y = sign(cmd_vel.linear.y) * vy_max;
  if (fabs(cmd_vel.angular.z) > vtheta_max_)
    cmd_vel.angular.z = sign(cmd_vel.angular.z) * vtheta_max_;

  // limit acceleration:
  // only acceleration (in terms of speeding up in any direction) is limited,
  // deceleration (in terms of slowing down) is handeled either by cob_teleop or the potential field
  // like slow-down behaviour above
  if (fabs(cmd_vel.linear.x) > fabs(vx_last_))
  {
    if ((cmd_vel.linear.x - vx_last_) / dt > ax_max_)
      cmd_vel.linear.x = vx_last_ + ax_max_ * dt;
    else if ((cmd_vel.linear.x - vx_last_) / dt < -ax_max_)
      cmd_vel.linear.x = vx_last_ - ax_max_ * dt;
  }
  if (fabs(cmd_vel.linear.y) > fabs(vy_last_))
  {
    if ((cmd_vel.linear.y - vy_last_) / dt > ay_max_)
      cmd_vel.linear.y = vy_last_ + ay_max_ * dt;
    else if ((cmd_vel.linear.y - vy_last_) / dt < -ay_max_)
      cmd_vel.linear.y = vy_last_ - ay_max_ * dt;
  }
  if (fabs(cmd_vel.angular.z) > fabs(vtheta_last_))
  {
    if ((cmd_vel.angular.z - vtheta_last_) / dt > atheta_max_)
      cmd_vel.angular.z = vtheta_last_ + atheta_max_ * dt;
    else if ((cmd_vel.angular.z - vtheta_last_) / dt < -atheta_max_)
      cmd_vel.angular.z = vtheta_last_ - atheta_max_ * dt;
  }

  pthread_mutex_lock(&m_mutex);
  vx_last_ = cmd_vel.linear.x;
  vy_last_ = cmd_vel.linear.y;
  vtheta_last_ = cmd_vel.angular.z;
  pthread_mutex_unlock(&m_mutex);

  velocity_limited_marker_.publishMarkers(cmd_vel_in.linear.x, cmd_vel.linear.x, cmd_vel_in.linear.y, cmd_vel.linear.y,
                                          cmd_vel_in.angular.z, cmd_vel.angular.z);

  // if closest obstacle is within stop_threshold, then do not move
  if (closest_obstacle_dist_ < stop_threshold_)
  {
    stopMovement();
  }
  else
  {
    // publish adjusted velocity
    topic_pub_command_.publish(cmd_vel);
  }
  return;
}

void CollisionVelocityFilter::obstacleHandler()
{
  pthread_mutex_lock(&m_mutex);
  closest_obstacle_dist_ = influence_radius_;
  pthread_mutex_unlock(&m_mutex);

  double cur_distance_to_center, cur_distance_to_border;
  double obstacle_theta_robot, obstacle_delta_theta_robot, obstacle_dist_vel_dir;
  bool cur_obstacle_relevant;
  geometry_msgs::Point cur_obstacle_robot;
  geometry_msgs::Point zero_position;
  zero_position.x = 0.0f;
  zero_position.y = 0.0f;
  zero_position.z = 0.0f;
  bool use_circumscribed = true, use_tube = true;

  //Calculate corner angles in robot_frame:
  double corner_front_left, corner_rear_left, corner_rear_right, corner_front_right;
  corner_front_left = atan2(footprint_left_, footprint_front_);
  corner_rear_left = atan2(footprint_left_, footprint_rear_);
  corner_rear_right = atan2(footprint_right_, footprint_rear_);
  corner_front_right = atan2(footprint_right_, footprint_front_);

  //Decide, whether circumscribed or tube argument should be used for filtering:
  if (fabs(robot_twist_linear_.x) <= 0.005f && fabs(robot_twist_linear_.y) <= 0.005f)
  {
    use_tube = false;
    //disable tube filter at very slow velocities
  }
  if (!use_tube)
  {
    if (fabs(robot_twist_angular_.z) <= 0.01f)
    {
      use_circumscribed = false;
    } //when tube filter inactive, start circumscribed filter at very low rot-velocities
  }
  else
  {
    if (fabs(robot_twist_angular_.z) <= use_circumscribed_threshold_)
    {
      use_circumscribed = false;
    } //when tube filter running, disable circum-filter in a wider range of rot-velocities
  }

  //Calculation of tube in driving-dir considered for obstacle avoidence
  double velocity_angle = 0.0f, velocity_ortho_angle;
  double corner_angle, delta_corner_angle;
  double ortho_corner_dist;
  double tube_left_border = 0.0f, tube_right_border = 0.0f;
  double tube_left_origin = 0.0f, tube_right_origin = 0.0f;
  double corner_dist, circumscribed_radius = 0.0f;

  for (unsigned i = 0; i < robot_footprint_.size(); i++)
  {
    corner_dist = sqrt(robot_footprint_[i].x * robot_footprint_[i].x + robot_footprint_[i].y * robot_footprint_[i].y);
    if (corner_dist > circumscribed_radius)
      circumscribed_radius = corner_dist;
  }

  if (use_tube)
  {
    //use commanded vel-value for vel-vector direction.. ?
    velocity_angle = atan2(robot_twist_linear_.y, robot_twist_linear_.x);
    velocity_ortho_angle = velocity_angle + M_PI / 2.0f;

    for (unsigned i = 0; i < robot_footprint_.size(); i++)
    {
      corner_angle = atan2(robot_footprint_[i].y, robot_footprint_[i].x);
      delta_corner_angle = velocity_ortho_angle - corner_angle;
      corner_dist = sqrt(robot_footprint_[i].x * robot_footprint_[i].x + robot_footprint_[i].y * robot_footprint_[i].y);
      ortho_corner_dist = cos(delta_corner_angle) * corner_dist;

      if (ortho_corner_dist < tube_right_border)
      {
        tube_right_border = ortho_corner_dist;
        tube_right_origin = sin(delta_corner_angle) * corner_dist;
      }
      else if (ortho_corner_dist > tube_left_border)
      {
        tube_left_border = ortho_corner_dist;
        tube_left_origin = sin(delta_corner_angle) * corner_dist;
      }
    }
  }

  //find relevant obstacles
  pthread_mutex_lock(&m_mutex);
  relevant_obstacles_.header.frame_id = global_frame_;
  relevant_obstacles_.header.stamp = ros::Time::now();
  relevant_obstacles_.data.clear();
  for (unsigned int i = 0;
      i
          < anti_collision_costmap_->getCostmap()->getSizeInCellsX()
              * anti_collision_costmap_->getCostmap()->getSizeInCellsY(); i++)
  {
    if (anti_collision_costmap_->getCostmap()->getCharMap()[i] == -1)
    {
      relevant_obstacles_.data.push_back(-1);
    }
    else if (anti_collision_costmap_->getCostmap()->getCharMap()[i] < costmap_obstacle_treshold_)
    { // add trshold
      relevant_obstacles_.data.push_back(0);
    }
    else
    {

      // calculate cell in 2D space where robot is is point (0, 0)
      geometry_msgs::Point cell;
      cell.x = (i % (int)(anti_collision_costmap_->getCostmap()->getSizeInCellsX()))
          * anti_collision_costmap_->getCostmap()->getResolution()
          + anti_collision_costmap_->getCostmap()->getOriginX();
      cell.y = (i / (int)(anti_collision_costmap_->getCostmap()->getSizeInCellsX()))
          * anti_collision_costmap_->getCostmap()->getResolution()
          + anti_collision_costmap_->getCostmap()->getOriginY();
      cell.z = 0.0f;

      cur_obstacle_relevant = false;
      cur_distance_to_center = getDistance2d(zero_position, cell);
      //check whether current obstacle lies inside the circumscribed_radius of the robot -> prevent collisions while rotating
      if (use_circumscribed && cur_distance_to_center <= circumscribed_radius)
      {
        cur_obstacle_robot = cell;

        if (obstacleValid(cur_obstacle_robot.x, cur_obstacle_robot.y))
        {
          cur_obstacle_relevant = true;
          obstacle_theta_robot = atan2(cur_obstacle_robot.y, cur_obstacle_robot.x);
        }

        //for each obstacle, now check whether it lies in the tube or not:
      }
      else if (use_tube && cur_distance_to_center < influence_radius_)
      {
        cur_obstacle_robot = cell;

        if (obstacleValid(cur_obstacle_robot.x, cur_obstacle_robot.y))
        {
          obstacle_theta_robot = atan2(cur_obstacle_robot.y, cur_obstacle_robot.x);
          obstacle_delta_theta_robot = obstacle_theta_robot - velocity_angle;
          obstacle_dist_vel_dir = sin(obstacle_delta_theta_robot) * cur_distance_to_center;

          if (obstacle_dist_vel_dir <= tube_left_border && obstacle_dist_vel_dir >= tube_right_border)
          {
            //found obstacle that lies inside of observation tube

            if (sign(obstacle_dist_vel_dir) >= 0)
            {
              if (cos(obstacle_delta_theta_robot) * cur_distance_to_center >= tube_left_origin)
              {
                //relevant obstacle in tube found
                cur_obstacle_relevant = true;
              }
            }
            else
            { // obstacle in right part of tube
              if (cos(obstacle_delta_theta_robot) * cur_distance_to_center >= tube_right_origin)
              {
                //relevant obstacle in tube found
                cur_obstacle_relevant = true;
              }
            }
          }
        }
      }

      if (cur_obstacle_relevant)
      {
        ROS_DEBUG_STREAM_NAMED("obstacleHandler", "[cob_collision_velocity_filter] Detected an obstacle");
        //relevant obstacle in tube found
        relevant_obstacles_.data.push_back(100);

        //now calculate distance of current, relevant obstacle to robot
        if (obstacle_theta_robot >= corner_front_right && obstacle_theta_robot < corner_front_left)
        {
          //obstacle in front:
          cur_distance_to_border = cur_distance_to_center - fabs(footprint_front_) / fabs(cos(obstacle_theta_robot));
        }
        else if (obstacle_theta_robot >= corner_front_left && obstacle_theta_robot < corner_rear_left)
        {
          //obstacle left:
          cur_distance_to_border = cur_distance_to_center - fabs(footprint_left_) / fabs(sin(obstacle_theta_robot));
        }
        else if (obstacle_theta_robot >= corner_rear_left || obstacle_theta_robot < corner_rear_right)
        {
          //obstacle in rear:
          cur_distance_to_border = cur_distance_to_center - fabs(footprint_rear_) / fabs(cos(obstacle_theta_robot));
        }
        else
        {
          //obstacle right:
          cur_distance_to_border = cur_distance_to_center - fabs(footprint_right_) / fabs(sin(obstacle_theta_robot));
        }

        if (cur_distance_to_border < closest_obstacle_dist_)
        {
          closest_obstacle_dist_ = cur_distance_to_border;
          closest_obstacle_angle_ = obstacle_theta_robot;
        }
      }
      else
      {
        relevant_obstacles_.data.push_back(0);
      }
    }
  }
  pthread_mutex_unlock(&m_mutex);

  topic_pub_relevant_obstacles_.publish(relevant_obstacles_);
  ROS_DEBUG_STREAM_NAMED("obstacleHandler",
                         "[cob_collision_velocity_filter] closest_obstacle_dist_ = " << closest_obstacle_dist_);
}

double CollisionVelocityFilter::getDistance2d(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

double CollisionVelocityFilter::sign(double x)
{
  if (x >= 0.0f)
    return 1.0f;
  else
    return -1.0f;
}

bool CollisionVelocityFilter::obstacleValid(double x_obstacle, double y_obstacle)
{
  if (x_obstacle < footprint_front_ && x_obstacle > footprint_rear_ && y_obstacle > footprint_right_
      && y_obstacle < footprint_left_)
  {
    ROS_WARN("Found an obstacle inside robot_footprint: Skip!");
    return false;
  }

  return true;
}

void CollisionVelocityFilter::stopMovement()
{
  geometry_msgs::Twist stop_twist;
  stop_twist.linear.x = 0.0f;
  stop_twist.linear.y = 0.0f;
  stop_twist.linear.z = 0.0f;
  stop_twist.angular.x = 0.0f;
  stop_twist.angular.y = 0.0f;
  stop_twist.linear.z = 0.0f;
  topic_pub_command_.publish(stop_twist);
  vx_last_ = 0.0;
  vy_last_ = 0.0;
  vtheta_last_ = 0.0;
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "cob_collision_velocity_filter");

  // create nodeClass
  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS("anti_collision_costmap", tf);
  CollisionVelocityFilter collisionVelocityFilter(costmap);

  ros::spin();

  return 0;
}

