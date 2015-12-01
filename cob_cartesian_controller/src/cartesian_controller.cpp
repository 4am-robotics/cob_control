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
#include <math.h>
#include <algorithm>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <cob_srvs/SetString.h>

#include <cob_cartesian_controller/cartesian_controller.h>

bool CartesianController::initialize()
{
    ros::NodeHandle nh_private("~");

    if (!nh_.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    if(!nh_.getParam("root_frame", root_frame_))
    {
        ROS_ERROR("Parameter 'reference_frame' not set");
        return false;
    }

    if(!nh_.getParam("update_rate", update_rate_))
    {
        update_rate_ = 50.0;    //hz
    }

    /// Private Nodehandle
    if(!nh_private.getParam("target_frame", target_frame_))
    {
        ROS_WARN("Parameter 'target_frame' not set. Using default 'cartesian_target'");
        target_frame_ = DEFAULT_CARTESIAN_TARGET;
    }


    ROS_WARN("Waiting for Services...");
    start_tracking_ = nh_.serviceClient<cob_srvs::SetString>("frame_tracker/start_tracking");
    stop_tracking_ = nh_.serviceClient<std_srvs::Empty>("frame_tracker/stop_tracking");
    start_tracking_.waitForExistence();
    stop_tracking_.waitForExistence();
    tracking_ = false;

    trajectory_interpolator_.reset(new TrajectoryInterpolator(root_frame_, update_rate_));

    action_name_ = "cartesian_trajectory_action";
    as_.reset(new SAS_CartesianControllerAction_t(nh_, action_name_, false));
    as_->registerGoalCallback(boost::bind(&CartesianController::goalCallback, this));
    as_->registerPreemptCallback(boost::bind(&CartesianController::preemptCallback, this));
    as_->start();

    ROS_INFO("...done!");
    return true;
}


//ToDo: Use the ActionInterface of the FrameTracker instead in order to be able to consider TrackingConstraints
bool CartesianController::startTracking()
{
    bool success = false;
    cob_srvs::SetString start;
    start.request.data = target_frame_;
    if(!tracking_)
    {
        success = start_tracking_.call(start);

        if(success)
        {
            success = start.response.success;
            if(success)
            {
                ROS_INFO("Response 'start_tracking': succeded");
                tracking_ = true;
            }
            else
            {
                ROS_ERROR("Response 'start_tracking': failed");
            }
        }
        else
        {
            ROS_ERROR("Failed to call service 'start_tracking'");
        }
    }
    else
    {
        ROS_WARN("Already tracking");
    }

    return success;
}

//ToDo:: If we use the ActionInterface of the FrameTracker, here that action should be cancled()
bool CartesianController::stopTracking()
{
    bool success = false;
    std_srvs::Empty stop;
    if(tracking_)
    {
        success = stop_tracking_.call(stop);

        if(success)
        {
            ROS_INFO("Service 'stop' succeded!");
            tracking_ = false;
        }
        else
        {
            ROS_ERROR("Failed to call service 'stop_tracking'");
        }
    }
    else
    {
        ROS_WARN("Have not been tracking");
    }

    return success;
}


// MovePTP
bool CartesianController::movePTP(const geometry_msgs::Pose& target_pose, const double epsilon)
{
    ROS_WARN("TEST");
    bool success = false;
    int reached_pos_counter = 0;

    ros::Rate rate(update_rate_);
    tf::StampedTransform stamped_transform;
    stamped_transform.setOrigin( tf::Vector3(target_pose.position.x,
                                             target_pose.position.y,
                                             target_pose.position.z) );
    stamped_transform.setRotation( tf::Quaternion(target_pose.orientation.x,
                                                  target_pose.orientation.y,
                                                  target_pose.orientation.z,
                                                  target_pose.orientation.w) );
    stamped_transform.frame_id_ = root_frame_;
    stamped_transform.child_frame_id_ = target_frame_;

    while(as_->isActive())
    {
        // Send/Refresh target Frame
        stamped_transform.stamp_ = ros::Time::now();
        tf_broadcaster_.sendTransform(stamped_transform);

        // Get transformation
        tf::StampedTransform stamped_transform = utils_.getStampedTransform(target_frame_, chain_tip_link_);

        // Wait for chain_tip_link to be within epsilon area of target_frame
        if(utils_.inEpsilonArea(stamped_transform, epsilon))
        {
            reached_pos_counter++;    // Count up if end effector position is in the epsilon area to avoid wrong values
        }

        if(reached_pos_counter >= static_cast<int>(2*update_rate_)) //has been close enough to target for 2 seconds
        {
            success = true;
            break;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return success;
}

// Broadcasting interpolated Cartesian path
bool CartesianController::posePathBroadcaster(const geometry_msgs::PoseArray& cartesian_path)
{
    bool success = true;
    double epsilon = 0.1;
    int failure_counter = 0;
    ros::Rate rate(update_rate_);
    tf::Transform transform;
    for(unsigned int i = 0; i < cartesian_path.poses.size(); i++)
    {
        if(!as_->isActive())
        {
            success = false;
            break;
        }

        // Send/Refresh target Frame
        transform.setOrigin( tf::Vector3(cartesian_path.poses.at(i).position.x,
                                         cartesian_path.poses.at(i).position.y,
                                         cartesian_path.poses.at(i).position.z) );
        transform.setRotation( tf::Quaternion(cartesian_path.poses.at(i).orientation.x,
                                              cartesian_path.poses.at(i).orientation.y,
                                              cartesian_path.poses.at(i).orientation.z,
                                              cartesian_path.poses.at(i).orientation.w) );

//        geometry_msgs::Transform tf_msg;
//        tf::transformTFToMsg(transform, tf_msg);
//        ROS_INFO_STREAM("pathArray[" << i << "]: " << tf_msg.rotation);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), cartesian_path.header.frame_id, target_frame_));


        // Get transformation
        tf::StampedTransform stamped_transform = utils_.getStampedTransform(target_frame_, chain_tip_link_);

        // Check whether chain_tip_link is within epsilon area of target_frame
//        if(!utils_.inEpsilonArea(stamped_transform, epsilon))
//        {
//            failure_counter++;
//        }
//        else
//        {
//            if(failure_counter > 0)
//            {
//                failure_counter--;
//            }
//        }

        //ToDo: This functionality is already implemented in the frame_tracker
        //Use the the ActionInterface of the FrameTracker instead of the ServiceCalls and modify constraints in FrameTracker accordingly
//        if(failure_counter > 100)
//        {
//            ROS_ERROR("Endeffector failed to track target_frame!");
//            success = false;
//            break;
//        }
        ros::spinOnce();
        rate.sleep();
    }

    while(true)
    {
        // Get transformation
        tf::StampedTransform stamped_transform = utils_.getStampedTransform(target_frame_, chain_tip_link_);
        ROS_INFO_STREAM("stamped: x: " << stamped_transform.getOrigin().getX() << " y: " << stamped_transform.getOrigin().getY() << " z: " << stamped_transform.getOrigin().getZ());

        if(!utils_.inEpsilonArea(stamped_transform, 0.0005))
        {
            // Send/Refresh target Frame
            transform.setOrigin( tf::Vector3(cartesian_path.poses.back().position.x,
                                             cartesian_path.poses.back().position.y,
                                             cartesian_path.poses.back().position.z) );
            transform.setRotation( tf::Quaternion(cartesian_path.poses.back().orientation.x,
                                                  cartesian_path.poses.back().orientation.y,
                                                  cartesian_path.poses.back().orientation.z,
                                                  cartesian_path.poses.back().orientation.w) );
            tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), cartesian_path.header.frame_id, target_frame_));
        }
        else
        {
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return success;
}


void CartesianController::goalCallback()
{
    geometry_msgs::Pose start_pose, end_pose, pose;
    geometry_msgs::PoseArray cartesian_path;
    cob_cartesian_controller::CartesianActionStruct action_struct;


    ROS_INFO_STREAM("=========================================================================");
    ROS_INFO("Received a new goal");
    ROS_INFO_STREAM("=========================================================================");

    start_pose = utils_.getPose(root_frame_, chain_tip_link_);

    action_struct = acceptGoal(as_->acceptNewGoal());

    if(action_struct.move_type == cob_cartesian_controller::CartesianControllerGoal::LIN)
    {
        ROS_INFO("move_lin");

        // Interpolate path
        if(!trajectory_interpolator_->linearInterpolation(cartesian_path, action_struct))
        {
            actionAbort(false, "Failed to do interpolation for 'move_lin'");
            return;
        }

        // Publish Preview
        utils_.previewPath(cartesian_path);

        // Activate Tracking
        if(!startTracking())
        {
            actionAbort(false, "Failed to start tracking");
            return;
        }

        // Execute path
        if(!posePathBroadcaster(cartesian_path))
        {
            actionAbort(false, "Failed to execute path for 'move_lin'");
            return;
        }

        movePTP(action_struct.move_lin.end, 0.001);

        // De-Activate Tracking
        if(!stopTracking())
        {
            actionAbort(false, "Failed to stop tracking");
            return;
        }

        actionSuccess(true, "move_lin succeeded!");
    }
    else if(action_struct.move_type == cob_cartesian_controller::CartesianControllerGoal::CIRC)
    {
        ROS_INFO("move_circ");
//        if(!trajectory_interpolator_->circularInterpolation(cartesian_path, action_struct))
//        {
//            actionAbort(false, "Failed to do interpolation for 'move_circ'");
//            return;
//        }
//
//        // Publish Preview
//        utils_.previewPath(cartesian_path);
//
//        // Activate Tracking
//        if(!startTracking())
//        {
//            actionAbort(false, "Failed to start tracking");
//            return;
//        }
//
//        // Move to start
//        if(!movePTP(cartesian_path.poses.at(0), 0.03))
//        {
//            actionAbort(false, "Failed to movePTP to start");
//            return;
//        }
//
//        // Execute path
//        if(!posePathBroadcaster(cartesian_path))
//        {
//            actionAbort(false, "Failed to execute path for 'move_circ'");
//            return;
//        }
//
//        // De-Activate Tracking
//        if(!stopTracking())
//        {
//            actionAbort(false, "Failed to stop tracking");
//            return;
//        }
//
//        actionSuccess(true, "move_circ succeeded!");
        actionAbort(false, "Not implemented.");
    }
    else
    {
        actionAbort(false, "Unknown trajectory action");
        return;
    }
}

cob_cartesian_controller::MoveLinStruct CartesianController::convertMoveLin(const cob_cartesian_controller::MoveLin& move_lin_msg)
{
    geometry_msgs::Pose start, end;
    start = utils_.getPose(root_frame_, chain_tip_link_);   //current tcp pose
    utils_.transformPose(move_lin_msg.frame_id, root_frame_, move_lin_msg.pose_goal, end);

    cob_cartesian_controller::MoveLinStruct move_lin;

    move_lin.start = start;
    move_lin.end = end;

    ROS_WARN("move_lin_msg_orientation .. w: %f", move_lin_msg.pose_goal.orientation.w);
    ROS_WARN("move_lin_msg_orientation .. x: %f", move_lin_msg.pose_goal.orientation.x);
    ROS_WARN("move_lin_msg_orientation .. y: %f", move_lin_msg.pose_goal.orientation.y);
    ROS_WARN("move_lin_msg_orientation .. z: %f", move_lin_msg.pose_goal.orientation.z);
    return move_lin;
}

cob_cartesian_controller::MoveCircStruct CartesianController::convertMoveCirc(const cob_cartesian_controller::MoveCirc& move_circ_msg)
{
    geometry_msgs::Pose center;
    utils_.transformPose(move_circ_msg.frame_id, root_frame_, move_circ_msg.pose_center, center);

    cob_cartesian_controller::MoveCircStruct move_circ;
    move_circ.start_angle           = move_circ_msg.start_angle;
    move_circ.end_angle             = move_circ_msg.end_angle;
    move_circ.radius                = move_circ_msg.radius;


    move_circ.pose_center = center;

    return move_circ;
}

cob_cartesian_controller::CartesianActionStruct CartesianController::acceptGoal(boost::shared_ptr<const cob_cartesian_controller::CartesianControllerGoal> goal)
{
    cob_cartesian_controller::CartesianActionStruct action_struct;
    action_struct.move_type = goal->move_type;

    action_struct.profile.vel           = goal->profile.vel;
    action_struct.profile.accl          = goal->profile.accl;
    action_struct.profile.profile_type  = goal->profile.profile_type;
    action_struct.profile.t_ipo         = 1/update_rate_;


    if(action_struct.move_type == cob_cartesian_controller::CartesianControllerGoal::LIN)
    {
        action_struct.move_lin = convertMoveLin(goal->move_lin);
    }
    else if(action_struct.move_type == cob_cartesian_controller::CartesianControllerGoal::CIRC)
    {
        action_struct.move_circ = convertMoveCirc(goal->move_circ);
    }
    else
    {

        actionAbort(false, "Unknown trajectory action " + boost::lexical_cast<std::string>(action_struct.move_type));
    }

    return action_struct;
}

void CartesianController::preemptCallback()
{
    // De-Activate Tracking
    stopTracking();
    actionPreempt(true, "action preempted!");
}

void CartesianController::actionSuccess(const bool success, const std::string& message)
{
    ROS_INFO_STREAM("Goal succeeded: " << message);
    action_result_.success = success;
    action_result_.message = message;
    as_->setSucceeded(action_result_, action_result_.message);
}

void CartesianController::actionPreempt(const bool success, const std::string& message)
{
    ROS_WARN_STREAM("Goal preempted: " << message);
    action_result_.success = success;
    action_result_.message = message;
    as_->setPreempted(action_result_, action_result_.message);
}

void CartesianController::actionAbort(const bool success, const std::string& message)
{
    ROS_ERROR_STREAM("Goal aborted: "  << message);
    action_result_.success = success;
    action_result_.message = message;
    as_->setAborted(action_result_, action_result_.message);
    stopTracking();
}
