/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_twist_action
 *
 * \author
 *   Author: Christian Ehrmann, email: christian.ehrmann@ipa.fraunhofer.de
 *
 * \date Date of creation: November, 2014
 *
 * \brief
 *   ...
 *
 ****************************************************************/
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Twist.h>
#include <cob_twist_action/cob_twist_actionConfig.h>
#include <cob_twist_action/TwistAction.h>
#include <cob_twist_action/TwistGoal.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_twist_action");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<cob_twist_action::TwistAction> ac("torso/perform_twist", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  cob_twist_action::TwistGoal goal;
  //geometry_msgs::Twist goal_twist;
  //goal_twist.linear.x = 0.0;
  //goal.twist_goal = goal_twist;
  bool active = true;
  goal.tracking = active;
  ac.sendGoal(goal);
  
  for (int i=0; i<5; i++) {
	  ros::Duration(1.0).sleep();
	  ROS_INFO("Wait some time!");
  }
  ROS_INFO("Cancel all goals!");
  ac.cancelAllGoals();
  //wait for the action to return
  ROS_INFO("Wait for action to timeout)!");
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
