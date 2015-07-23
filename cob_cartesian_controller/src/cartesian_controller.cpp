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

#include <ros/ros.h>
#include "ros/package.h"
#include <std_srvs/Empty.h>
#include <cob_srvs/SetString.h>
#include <kdl_conversions/kdl_msg.h>

#include <cob_cartesian_controller/cartesian_controller.h>
#include <cob_cartesian_controller/trajectory_interpolator/trajectory_interpolator.h>

bool CartesianController::initialize()
{
    ros::NodeHandle nh_private("~");

    ///get params articulation Nodehandle
    if(!nh_private.getParam("reference_frame", reference_frame_))
    {
        ROS_ERROR("Parameter 'reference_frame' not set");
        return false;
    }

    if(!nh_private.getParam("target_frame", target_frame_))
    {
        ROS_ERROR("Parameter 'target_frame' not set");
        return false;
    }

    if (nh_private.hasParam("update_rate"))
    {    nh_private.getParam("update_rate", update_rate_);    }
    else
    {    update_rate_ = 68.0;    }    //hz


    /// Cartesian Nodehandle
    if (!nh_.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    ROS_WARN("Waiting for Services...");
    start_tracking_ = nh_.serviceClient<cob_srvs::SetString>("frame_tracker/start_tracking");
    stop_tracking_ = nh_.serviceClient<std_srvs::Empty>("frame_tracker/stop_tracking");
    start_tracking_.waitForExistence();
    stop_tracking_.waitForExistence();

    action_name_ = "cartesian_trajectory_action_";
    as_.reset(new SAS_CartesianControllerAction_t(nh_, action_name_, false));
    as_->registerGoalCallback(boost::bind(&CartesianController::goalCB, this));
    as_->registerPreemptCallback(boost::bind(&CartesianController::preemptCB, this));
    as_->start();

    tracking_goal_ = false;
    tracking_ = false;
    failure_counter_ = 0;

    ROS_INFO("...done!");
    return true;
}

void CartesianController::run()
{
    ros::Rate r(100.0);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}

void CartesianController::timerCallback(const ros::TimerEvent& event)
{
    hold_ = false;
}

// Pseudo PTP
void CartesianController::movePTP(geometry_msgs::Pose target_pose, double epsilon)
{
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::TransformListener listener;
    reached_pos_ = false;
    int reached_pos_counter = 0;
    double ro, pi, ya;
    ros::Rate rate(update_rate_);
    tf::StampedTransform stamped_transform;
    tf::Quaternion q;

    while(ros::ok())
    {
        // Linearkoordinaten
        transform.setOrigin( tf::Vector3(target_pose.position.x, target_pose.position.y, target_pose.position.z) );

        q = tf::Quaternion(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
        transform.setRotation(q);

        // Send br Frame
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), reference_frame_, target_frame_));

        // Get transformation
        try
        {
            startTracking();
            listener.lookupTransform(target_frame_, chain_tip_link_, ros::Time(0), stamped_transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        // Get current RPY out of quaternion
        tf::Quaternion quatern = stamped_transform.getRotation();
        tf::Matrix3x3(quatern).getRPY(ro,pi,ya);


        // Wait for arm_7_link to be in position
        if(utils_.epsilonArea(stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z(), ro, pi, ya,epsilon))
        {
            reached_pos_counter++;    // Count up if end effector position is in the epsilon area to avoid wrong values
        }

        if(reached_pos_counter >= 50)
        {
            reached_pos_ = true;
        }

        if(reached_pos_ == true)    // Cancel while loop
        {
            break;
        }
        rate.sleep();
        ros::spinOnce();
    }
}

void CartesianController::posePathBroadcaster(std::vector<geometry_msgs::Pose>& pose_vector)
{
    double roll, pitch, yaw;
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    ros::Rate rate(update_rate_);
    tf::Quaternion q;
    double T_IPO = pow(update_rate_, -1);
    double epsilon = 0.1;

    startTracking();

    for(int i = 0; i < pose_vector.size()-1; i++)
    {
        if(!tracking_goal_)
        {
            throw errorException("Action was preempted.");
        }

        ros::Time now = ros::Time::now();

        // Linearkoordinaten
        transform.setOrigin(tf::Vector3(pose_vector.at(i).position.x, pose_vector.at(i).position.y, pose_vector.at(i).position.z));
        q = tf::Quaternion(pose_vector.at(i).orientation.x,
                           pose_vector.at(i).orientation.y,
                           pose_vector.at(i).orientation.z,
                           pose_vector.at(i).orientation.w);
        transform.setRotation(q);

//        utils_.showMarker(tf::StampedTransform(transform, ros::Time::now(), reference_frame_, target_frame_), reference_frame_, marker_, 0 , 1, 0, "goalFrame");

        br.sendTransform(tf::StampedTransform(transform, now, reference_frame_, target_frame_));

        marker_++;

        // Get transformation
        try
        {
            listener.lookupTransform(target_frame_, chain_tip_link_, ros::Time(0), stamped_transform);
        }
        catch (tf::TransformException& ex)
        {
//            ROS_ERROR("%s",ex.what());
        }

        // Get current RPY out of quaternion
        tf::Quaternion quatern = stamped_transform.getRotation();
        tf::Matrix3x3(quatern).getRPY(roll, pitch, yaw);

        if(!utils_.epsilonArea(stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z(),
                                roll, pitch, yaw, epsilon))
        {
            failure_counter_++;
        }
        else
        {
            if(failure_counter_ > 0)
            {
                failure_counter_--;
            }
        }

        if(failure_counter_ > 100)
        {
            stopTracking();
            throw errorException("Distance between endeffector and tracking_frame exceeded the limit.");
        }

        ros::spinOnce();
        rate.sleep();
    }

    stopTracking();
}

void CartesianController::holdPosition(geometry_msgs::Pose hold_pose)
{
    tf::TransformBroadcaster br;
    tf::Transform transform;
    ros::Rate rate(update_rate_);
    tf::Quaternion q;

    startTracking();

    while(hold_)
    {
        // Linearcoordinates
        transform.setOrigin( tf::Vector3(hold_pose.position.x, hold_pose.position.y, hold_pose.position.z) );

        // RPY Angles
        q = tf::Quaternion(hold_pose.orientation.x, hold_pose.orientation.y, hold_pose.orientation.z, hold_pose.orientation.w);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), reference_frame_, target_frame_));
        ros::spinOnce();
        rate.sleep();
    }

    stopTracking();
}

void CartesianController::startTracking()
{
    cob_srvs::SetString start;
    start.request.data = target_frame_;
    if(!tracking_)
    {
        start_tracking_.call(start);

        if(start.response.success==true)
        {
            ROS_INFO("...service called!");
            tracking_ = true;
        }
        else
        {
            ROS_INFO("...service failed");
        }
    }

}

void CartesianController::stopTracking()
{
    std_srvs::Empty srv_save_stop;
    srv_save_stop.request;
    if(tracking_)
    {
        if(stop_tracking_.call(srv_save_stop))
        {
            ROS_INFO("... service stopped!");
            tracking_ = false;
        }
        else
        {
            ROS_ERROR("... service stop failed! FATAL!");
        }
    }
}

void CartesianController::goalCB()
{
    tf::TransformListener listener;
    TrajectoryInterpolator TIP(update_rate_);
    std::vector <geometry_msgs::Pose> pos_vec;
    geometry_msgs::Pose actual_tcp_pose;
    double roll, pitch, yaw;
    cob_cartesian_controller::CartesianActionStruct action_struct;

    ROS_INFO("Received a new goal");

    if (as_->isNewGoalAvailable())
    {
        action_struct = acceptGoal(as_->acceptNewGoal());
        tracking_goal_ = true;

        if(action_struct.name == "move_lin")
        {
            ROS_INFO("move_lin");

            cob_cartesian_controller::MoveLinStruct move_lin = convertMoveLinRelToAbs(action_struct.move_lin);

            // Interpolate the path
            if(TIP.linearInterpolation(pos_vec, move_lin))
            {
                //Broadcast the linear path
                try
                {
                    posePathBroadcaster(pos_vec);
                    actionSuccess();
                }
                catch (errorException& e)
                {
                    ROS_WARN("%s",e.what());
                    actionAbort();
                }
            }
            else
            {
                ROS_ERROR("Error while interpolation linear path.");
                tracking_goal_ = false;
            }
        }
        else if(action_struct.name == "move_circ")
        {
            ROS_INFO("move_circ");
            cob_cartesian_controller::MoveCircStruct move_circ = convertMoveCircRelToAbs(action_struct.move_circ);

            if(!TIP.circularInterpolation(pos_vec, move_circ))
            {
                movePTP(pos_vec.at(0), 0.03);
                //Broadcast the circular path
                try
                {
                    posePathBroadcaster(pos_vec);
                    actionSuccess();
                }
                catch (errorException& e)
                {
                    ROS_WARN("%s",e.what());
                    actionAbort();
                }

                actual_tcp_pose = pos_vec.at(pos_vec.size()-1);
            }
            else
            {
                ROS_ERROR("Error while interpolation circular path.");
                tracking_goal_ = false;
            }
        }
        else if(action_struct.name == "hold")
        {
            ROS_INFO("Hold position");

            actual_tcp_pose = utils_.getEndeffectorPose(listener, reference_frame_, chain_tip_link_);
            ros::Timer timer = nh_.createTimer(ros::Duration(action_struct.hold_time), &CartesianController::timerCallback, this);
            hold_ = true;

            holdPosition(actual_tcp_pose);
            actionSuccess();
        }
        else
        {
            ROS_ERROR("Unknown trajectory action");
            actionAbort();
        }
        pos_vec.clear();
    }
}


cob_cartesian_controller::MoveLinStruct CartesianController::convertMoveLinRelToAbs(const cob_cartesian_controller::MoveLinStruct& rel_move_lin)
{
    tf::TransformListener listener;
    geometry_msgs::Pose actualTcpPose, end;
    tf::Quaternion q_start,q_end, q_rel;
    actualTcpPose = utils_.getEndeffectorPose(listener, reference_frame_, chain_tip_link_);

    cob_cartesian_controller::MoveLinStruct update_ml = rel_move_lin;

    // Transform RPY to Quaternion
    q_rel.setRPY(rel_move_lin.roll, rel_move_lin.pitch, rel_move_lin.yaw);

    q_start = tf::Quaternion(actualTcpPose.orientation.x,
                             actualTcpPose.orientation.y,
                             actualTcpPose.orientation.z,
                             actualTcpPose.orientation.w);

    // q_end = q_start * q_rel; // this does a rotation with respect to the endeffector but the rotation should be relative to the reference frame
    q_end = q_rel * q_start; // this does a rotation with respect to the reference as well as the lin movement is done in abs. coordinates

    // Define End Pose
    end.position.x = actualTcpPose.position.x + rel_move_lin.x;
    end.position.y = actualTcpPose.position.y + rel_move_lin.y;
    end.position.z = actualTcpPose.position.z + rel_move_lin.z;
    end.orientation.x = q_end.getX();
    end.orientation.y = q_end.getY();
    end.orientation.z = q_end.getZ();
    end.orientation.w = q_end.getW();

    utils_.poseToRPY(actualTcpPose, update_ml.roll, update_ml.pitch, update_ml.yaw);
    update_ml.start = actualTcpPose;
    update_ml.end = end;

    return update_ml;
}


cob_cartesian_controller::MoveCircStruct CartesianController::convertMoveCircRelToAbs(cob_cartesian_controller::MoveCircStruct& rel_move_circ)
{
    return rel_move_circ;
}


void CartesianController::preemptCB()
{
    ROS_INFO("Received a preemption request");
    action_result_.success = true;
    action_result_.message = "Action has been preempted";
    as_->setPreempted(action_result_);

    ROS_WARN("Action was preempted");

    tracking_goal_ = false;
    stopTracking();
}


cob_cartesian_controller::CartesianActionStruct CartesianController::acceptGoal(boost::shared_ptr<const cob_cartesian_controller::CartesianControllerGoal> goal)
{
    cob_cartesian_controller::CartesianActionStruct action_struct;
    action_struct.name = goal->name;

    if(action_struct.name == "move_lin")
    {
        action_struct.move_lin.x           = goal->move_lin.x;
        action_struct.move_lin.y           = goal->move_lin.y;
        action_struct.move_lin.z           = goal->move_lin.z;
        action_struct.move_lin.roll        = goal->move_lin.roll * M_PI / 180.0;
        action_struct.move_lin.pitch       = goal->move_lin.pitch * M_PI / 180.0;
        action_struct.move_lin.yaw         = goal->move_lin.yaw * M_PI / 180.0;
        action_struct.move_lin.rotate_only = goal->move_lin.rotate_only;
        action_struct.move_lin.profile.vel          = goal->move_lin.profile.vel;
        action_struct.move_lin.profile.accl         = goal->move_lin.profile.accl;
        action_struct.move_lin.profile.profile_type = goal->move_lin.profile.profile_type;
    }
    else if(action_struct.name == "move_circ")
    {
        action_struct.move_circ.x_center      = goal->move_circ.x_center;
        action_struct.move_circ.y_center      = goal->move_circ.y_center;
        action_struct.move_circ.z_center      = goal->move_circ.z_center;
        action_struct.move_circ.roll_center   = goal->move_circ.roll_center * M_PI / 180.0;
        action_struct.move_circ.pitch_center  = goal->move_circ.pitch_center * M_PI / 180.0;
        action_struct.move_circ.yaw_center    = goal->move_circ.yaw_center * M_PI / 180.0;
        action_struct.move_circ.start_angle   = goal->move_circ.start_angle;
        action_struct.move_circ.end_angle     = goal->move_circ.end_angle;
        action_struct.move_circ.radius        = goal->move_circ.radius;
        action_struct.move_circ.profile.vel           = goal->move_circ.profile.vel;
        action_struct.move_circ.profile.accl          = goal->move_circ.profile.accl;
        action_struct.move_circ.profile.profile_type  = goal->move_circ.profile.profile_type;
    }
    else if(action_struct.name == "hold")
    {
        action_struct.hold_time = goal ->hold_time;
    }
    else
    {
        ROS_ERROR_STREAM("There is no handling for the action with name: " << action_struct.name);
        actionAbort();
    }

    return action_struct;
}


void CartesianController::actionSuccess()
{
    as_->setSucceeded(action_result_, action_result_.message);
    ROS_INFO("Goal succeeded!");
}


void CartesianController::actionAbort()
{
    ROS_INFO("Goal aborted");
    as_->setAborted(action_result_, action_result_.message);
    tracking_goal_ = false;
}
