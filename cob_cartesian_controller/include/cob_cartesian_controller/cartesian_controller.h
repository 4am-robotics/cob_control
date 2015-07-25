/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
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
 *   ROS package name: cob_cartesian_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: July, 2015
 *
 * \brief
 *   ...
 *
 ****************************************************************/

#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H

#include <ros/ros.h>
#include <vector>
#include <string.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <actionlib/server/simple_action_server.h>
#include <cob_cartesian_controller/CartesianControllerAction.h>

#include <cob_cartesian_controller/cartesian_controller_data_types.h>
#include <cob_cartesian_controller/cartesian_controller_utils.h>

typedef actionlib::SimpleActionServer<cob_cartesian_controller::CartesianControllerAction> SAS_CartesianControllerAction_t;

class CartesianController
{
public:
    bool initialize();
    
    // Main functions
    void posePathBroadcaster(std::vector<geometry_msgs::Pose>& pose_vector);
    void movePTP(geometry_msgs::Pose target_pose, double epsilon);
    
    // Helper function
    void startTracking();
    void stopTracking();

    /// Action interface
    void goalCB();
    void preemptCB();
    void actionSuccess();
    void actionAbort();
    cob_cartesian_controller::CartesianActionStruct acceptGoal(boost::shared_ptr<const cob_cartesian_controller::CartesianControllerGoal> goal);

    cob_cartesian_controller::MoveLinStruct convertMoveLinRelToAbs(const cob_cartesian_controller::MoveLinStruct& rel_move_lin);
    cob_cartesian_controller::MoveCircStruct convertMoveCircRelToAbs(cob_cartesian_controller::MoveCircStruct& rel_move_circ);

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::ServiceClient start_tracking_;
    ros::ServiceClient stop_tracking_;

    double update_rate_;
    std::string root_frame_, chain_tip_link_, target_frame_;
    
    // HelperVars for movePTP
    bool reached_pos_;

    /// Action interface
    std::string action_name_;
    boost::shared_ptr<SAS_CartesianControllerAction_t> as_;
    cob_cartesian_controller::CartesianControllerFeedback action_feedback_;
    cob_cartesian_controller::CartesianControllerResult action_result_;

    CartesianControllerUtils utils_;

    bool tracking_;
    bool tracking_goal_;
    double distance_;
    double failure_counter_;
};

#endif
