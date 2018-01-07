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


#include <math.h>
#include <algorithm>
#include <string>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
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

    if (!nh_.getParam("root_frame", root_frame_))
    {
        ROS_ERROR("Parameter 'reference_frame' not set");
        return false;
    }

    if (!nh_.getParam("update_rate", update_rate_))
    {
        update_rate_ = 50.0;  // hz
    }

    /// Private Nodehandle
    if (!nh_private.getParam("target_frame", target_frame_))
    {
        ROS_WARN("Parameter 'target_frame' not set. Using default 'cartesian_target'");
        target_frame_ = DEFAULT_CARTESIAN_TARGET;
    }

    ROS_WARN("Waiting for Services...");
    start_tracking_ = nh_.serviceClient<cob_srvs::SetString>("frame_tracker/start_tracking");
    stop_tracking_ = nh_.serviceClient<std_srvs::Trigger>("frame_tracker/stop");
    start_tracking_.waitForExistence();
    stop_tracking_.waitForExistence();
    tracking_ = false;

    trajectory_interpolator_.reset(new TrajectoryInterpolator(root_frame_, update_rate_));

    action_name_ = "cartesian_trajectory_action";
    as_.reset(new SAS_CartesianControllerAction_t(nh_, action_name_, false));
    as_->registerGoalCallback(boost::bind(&CartesianController::goalCallback, this));
    as_->registerPreemptCallback(boost::bind(&CartesianController::preemptCallback, this));
    as_->start();

    ROS_INFO("Cartesian Controller running");
    return true;
}

// ToDo: Use the ActionInterface of the FrameTracker instead in order to be able to consider TrackingConstraints
bool CartesianController::startTracking()
{
    bool success = false;
    cob_srvs::SetString start;
    start.request.data = target_frame_;
    if (!tracking_)
    {
        success = start_tracking_.call(start);

        if (success)
        {
            success = start.response.success;
            if (success)
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

// ToDo:: If we use the ActionInterface of the FrameTracker, here that action should be cancled()
bool CartesianController::stopTracking()
{
    bool success = false;
    std_srvs::Trigger stop;
    if (tracking_)
    {
        success = stop_tracking_.call(stop);

        if (success)
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

// Broadcasting interpolated Cartesian path
bool CartesianController::posePathBroadcaster(const geometry_msgs::PoseArray& cartesian_path)
{
    bool success = true;
    ros::Rate rate(update_rate_);
    tf::Transform transform;

    for (unsigned int i = 0; i < cartesian_path.poses.size(); i++)
    {
        if (!as_->isActive())
        {
            success = false;
            break;
        }

        // Send/Refresh target Frame
        transform.setOrigin(tf::Vector3(cartesian_path.poses.at(i).position.x,
                                        cartesian_path.poses.at(i).position.y,
                                        cartesian_path.poses.at(i).position.z));
        transform.setRotation(tf::Quaternion(cartesian_path.poses.at(i).orientation.x,
                                             cartesian_path.poses.at(i).orientation.y,
                                             cartesian_path.poses.at(i).orientation.z,
                                             cartesian_path.poses.at(i).orientation.w));

        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), cartesian_path.header.frame_id, target_frame_));

        ros::spinOnce();
        rate.sleep();
    }

    return success;
}


void CartesianController::goalCallback()
{
    geometry_msgs::PoseArray cartesian_path;
    cob_cartesian_controller::CartesianActionStruct action_struct;

    action_struct = acceptGoal(as_->acceptNewGoal());

    if (action_struct.move_type == cob_cartesian_controller::CartesianControllerGoal::LIN)
    {
        // Interpolate path
        if (!trajectory_interpolator_->linearInterpolation(cartesian_path, action_struct))
        {
            actionAbort(false, "Failed to do interpolation for 'move_lin'");
            return;
        }

        // Publish Preview
        utils_.previewPath(cartesian_path);
        
        // initially broadcast target_frame
        tf::Transform identity = tf::Transform();
        identity.setIdentity();
        tf_broadcaster_.sendTransform(tf::StampedTransform(identity, ros::Time::now(), chain_tip_link_, target_frame_));

        // Activate Tracking
        if (!startTracking())
        {
            actionAbort(false, "Failed to start tracking");
            return;
        }

        // Execute path
        if (!posePathBroadcaster(cartesian_path))
        {
            actionAbort(false, "Failed to execute path for 'move_lin'");
            return;
        }

        // De-Activate Tracking
        if (!stopTracking())
        {
            actionAbort(false, "Failed to stop tracking");
            return;
        }

        actionSuccess(true, "move_lin succeeded!");
    }
    else if (action_struct.move_type == cob_cartesian_controller::CartesianControllerGoal::CIRC)
    {
        if (!trajectory_interpolator_->circularInterpolation(cartesian_path, action_struct))
        {
            actionAbort(false, "Failed to do interpolation for 'move_circ'");
            return;
        }

        // Publish Preview
        utils_.previewPath(cartesian_path);

        // Activate Tracking
        if (!startTracking())
        {
            actionAbort(false, "Failed to start tracking");
            return;
        }

        // Execute path
        if (!posePathBroadcaster(cartesian_path))
        {
            actionAbort(false, "Failed to execute path for 'move_circ'");
            return;
        }

        // De-Activate Tracking
        if (!stopTracking())
        {
            actionAbort(false, "Failed to stop tracking");
            return;
        }

        actionSuccess(true, "move_circ succeeded!");
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
    start = utils_.getPose(root_frame_, chain_tip_link_);   // current tcp pose
    utils_.transformPose(move_lin_msg.frame_id, root_frame_, move_lin_msg.pose_goal, end);

    cob_cartesian_controller::MoveLinStruct move_lin;

    move_lin.start = start;
    move_lin.end = end;

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


    if (action_struct.move_type == cob_cartesian_controller::CartesianControllerGoal::LIN)
    {
        action_struct.move_lin = convertMoveLin(goal->move_lin);
    }
    else if (action_struct.move_type == cob_cartesian_controller::CartesianControllerGoal::CIRC)
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
