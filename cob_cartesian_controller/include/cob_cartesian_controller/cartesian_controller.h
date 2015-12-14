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
 * \date Date of creation: December, 2015
 *
 * \brief
 *   This class is used to interpolate various Cartesian paths for a given velocity profile.
 *   Supported types of Cartesian paths are Linear and Circular.
 *   Supported types of velocity profiles are Ramp and Sinoid.
 *
 ****************************************************************/

#ifndef COB_CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H
#define COB_CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <actionlib/server/simple_action_server.h>
#include <cob_cartesian_controller/CartesianControllerAction.h>

#include <cob_cartesian_controller/trajectory_interpolator/trajectory_interpolator.h>
#include <cob_cartesian_controller/cartesian_controller_data_types.h>
#include <cob_cartesian_controller/cartesian_controller_utils.h>

typedef actionlib::SimpleActionServer<cob_cartesian_controller::CartesianControllerAction> SAS_CartesianControllerAction_t;

#define DEFAULT_CARTESIAN_TARGET "cartesian_target"

class CartesianController
{
public:
    bool initialize();

    // Main functions
    bool posePathBroadcaster(const geometry_msgs::PoseArray& cartesian_path);

    // Helper function
    bool startTracking();
    bool stopTracking();

    /// Action interface
    void goalCallback();
    void preemptCallback();
    void actionSuccess(const bool success, const std::string& message);
    void actionPreempt(const bool success, const std::string& message);
    void actionAbort(const bool success, const std::string& message);

    cob_cartesian_controller::CartesianActionStruct acceptGoal(boost::shared_ptr<const cob_cartesian_controller::CartesianControllerGoal> goal);
    cob_cartesian_controller::MoveLinStruct convertMoveLin(const cob_cartesian_controller::MoveLin& move_lin_msg);
    cob_cartesian_controller::MoveCircStruct convertMoveCirc(const cob_cartesian_controller::MoveCirc& move_circ_msg);

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::ServiceClient start_tracking_;
    ros::ServiceClient stop_tracking_;
    bool tracking_;

    double update_rate_;
    std::string root_frame_, chain_tip_link_, target_frame_;

    /// Action interface
    std::string action_name_;
    boost::shared_ptr<SAS_CartesianControllerAction_t> as_;
    cob_cartesian_controller::CartesianControllerFeedback action_feedback_;
    cob_cartesian_controller::CartesianControllerResult action_result_;

    CartesianControllerUtils utils_;
    boost::shared_ptr< TrajectoryInterpolator > trajectory_interpolator_;
};

#endif  // COB_CARTESIAN_CONTROLLER_CARTESIAN_CONTROLLER_H
