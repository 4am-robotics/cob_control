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
    if(!nh_private.getParam("reference_frame", referenceFrame_))
    {
        ROS_ERROR("Parameter 'reference_frame' not set");
        return false;
    }

    if(!nh_private.getParam("target_frame", targetFrame_))
    {
        ROS_ERROR("Parameter 'target_frame' not set");
        return false;
    }

    if (nh_private.hasParam("update_rate"))
    {	nh_private.getParam("update_rate", update_rate_);	}
    else
    {	update_rate_ = 68.0;	}	//hz


    /// Cartesian Nodehandle
    if (!nh_.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    ROS_WARN("Waiting for Services...");
    startTracking_ = nh_.serviceClient<cob_srvs::SetString>("frame_tracker/start_tracking");
    stopTracking_ = nh_.serviceClient<std_srvs::Empty>("frame_tracker/stop_tracking");
    startTracking_.waitForExistence();
    stopTracking_.waitForExistence();

    action_name_ = "cartesian_trajectory_action_";
    as_.reset(new tSAS_CartesianControllerAction(nh_, action_name_, false));
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
void CartesianController::movePTP(geometry_msgs::Pose targetPose, double epsilon)
{
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::TransformListener listener;
    reached_pos_ = false;
    int reached_pos_counter = 0;
    double ro, pi, ya;
    ros::Rate rate(update_rate_);
    tf::StampedTransform stampedTransform;
    tf::Quaternion q;

    while(ros::ok())
    {
        // Linearkoordinaten
        transform.setOrigin( tf::Vector3(targetPose.position.x, targetPose.position.y, targetPose.position.z) );

        q = tf::Quaternion(targetPose.orientation.x, targetPose.orientation.y, targetPose.orientation.z, targetPose.orientation.w);
        transform.setRotation(q);

        // Send br Frame
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), referenceFrame_, targetFrame_));

        // Get transformation
        try
        {
            startTracking();
            listener.lookupTransform(targetFrame_, chain_tip_link_, ros::Time(0), stampedTransform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        // Get current RPY out of quaternion
        tf::Quaternion quatern = stampedTransform.getRotation();
        tf::Matrix3x3(quatern).getRPY(ro,pi,ya);


        // Wait for arm_7_link to be in position
        if(utils_.epsilon_area(stampedTransform.getOrigin().x(), stampedTransform.getOrigin().y(), stampedTransform.getOrigin().z(), ro, pi, ya,epsilon))
        {
            reached_pos_counter++;	// Count up if end effector position is in the epsilon area to avoid wrong values
        }

        if(reached_pos_counter >= 50)
        {
            reached_pos_ = true;
        }

        if(reached_pos_ == true)	// Cancel while loop
        {
            break;
        }
        rate.sleep();
        ros::spinOnce();
    }
}

void CartesianController::posePathBroadcaster(std::vector <geometry_msgs::Pose> *poseVector)
{
    double roll, pitch, yaw;
    tf::TransformListener listener;
    tf::StampedTransform stampedTransform;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    ros::Rate rate(update_rate_);
    tf::Quaternion q;
    double T_IPO = pow(update_rate_, -1);
    double epsilon = 0.1;

    startTracking();

    for(int i = 0; i < poseVector->size()-1; i++)
    {
        if(!tracking_goal_)
        {
            throw errorException("Action was preempted.");
        }

        ros::Time now = ros::Time::now();

        // Linearkoordinaten
        transform.setOrigin(tf::Vector3(poseVector->at(i).position.x, poseVector->at(i).position.y, poseVector->at(i).position.z));
        q = tf::Quaternion(poseVector->at(i).orientation.x,
                           poseVector->at(i).orientation.y,
                           poseVector->at(i).orientation.z,
                           poseVector->at(i).orientation.w);
        transform.setRotation(q);

//        utils_.showMarker(tf::StampedTransform(transform, ros::Time::now(), referenceFrame_, targetFrame_),referenceFrame_,marker1_, 0 , 1.0 , 0 ,"goalFrame");

        br.sendTransform(tf::StampedTransform(transform, now, referenceFrame_, targetFrame_));

        marker1_++;

        // Get transformation
        try
        {
            listener.lookupTransform(targetFrame_, chain_tip_link_, ros::Time(0), stampedTransform);
        }
        catch (tf::TransformException &ex)
        {
//            ROS_ERROR("%s",ex.what());
        }

        // Get current RPY out of quaternion
        tf::Quaternion quatern = stampedTransform.getRotation();
        tf::Matrix3x3(quatern).getRPY(roll, pitch, yaw);

        if(!utils_.epsilon_area(stampedTransform.getOrigin().x(), stampedTransform.getOrigin().y(), stampedTransform.getOrigin().z(),
                                roll, pitch, yaw, epsilon))
        {
            failure_counter_++;
        }
        else
        {
            if(failure_counter_ > 0)
                failure_counter_--;
        }

        if(failure_counter_ > 50)
        {
            stopTracking();
            throw errorException("Distance between endeffector and tracking_frame exceeded the limit.");
        }

        ros::spinOnce();
        rate.sleep();
    }

    stopTracking();
}

void CartesianController::holdPosition(geometry_msgs::Pose holdPose)
{
    tf::TransformBroadcaster br;
    tf::Transform transform;
    ros::Rate rate(update_rate_);
    tf::Quaternion q;

    startTracking();

    while(hold_)
    {
        // Linearcoordinates
        transform.setOrigin( tf::Vector3(holdPose.position.x,holdPose.position.y,holdPose.position.z) );

        // RPY Angles
        q = tf::Quaternion(holdPose.orientation.x,holdPose.orientation.y,holdPose.orientation.z,holdPose.orientation.w);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), referenceFrame_, targetFrame_));
        ros::spinOnce();
        rate.sleep();
    }

    stopTracking();
}

void CartesianController::startTracking()
{
    cob_srvs::SetString start;
    start.request.data = targetFrame_;
    if(!tracking_)
    {
        startTracking_.call(start);

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
        if(stopTracking_.call(srv_save_stop))
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
    std::vector <geometry_msgs::Pose> posVec;
    geometry_msgs::Pose actualTcpPose;
    double roll, pitch, yaw;
    trajectory_action ta;

    ROS_INFO("Received a new goal");

    if (as_->isNewGoalAvailable())
    {
        ta = acceptGoal(as_->acceptNewGoal());
        tracking_goal_ = true;

        if(ta.name == "move_lin")
        {
            ROS_INFO("move_lin");

            trajectory_action_move_lin ta_move_lin = convertActionIntoMoveLin(ta);

            // Interpolate the path
            if(TIP.linear_interpolation(posVec, ta_move_lin))
            {
                //Broadcast the linear path
                try
                {
                    posePathBroadcaster(&posVec);
                    actionSuccess();
                }
                catch (errorException &e) {
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
        else if(ta.name == "move_circ")
        {
            ROS_INFO("move_circ");
            trajectory_action_move_circ ta_move_circ = convertActionIntoMoveCirc(ta);

            if(!TIP.circular_interpolation(posVec, ta_move_circ))
            {
                movePTP(posVec.at(0), 0.03);
                //Broadcast the circular path
                try
                {
                    posePathBroadcaster(&posVec);
                    actionSuccess();
                }
                catch (errorException &e) {
                    ROS_WARN("%s",e.what());
                    actionAbort();
                }

                actualTcpPose = posVec.at(posVec.size()-1);
            }
            else
            {
                ROS_ERROR("Error while interpolation circular path.");
                tracking_goal_ = false;
            }
        }

        else if(ta.name == "hold")
        {
            ROS_INFO("Hold position");

            actualTcpPose = utils_.getEndeffectorPose(listener,referenceFrame_, chain_tip_link_);
            ros::Timer timer = nh_.createTimer(ros::Duration(ta.hold_time), &CartesianController::timerCallback, this);
            hold_ = true;

            holdPosition(actualTcpPose);
            actionSuccess();
        }
        else
        {
            ROS_ERROR("Unknown trajectory action");
            actionAbort();
        }
        posVec.clear();
    }
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


trajectory_action CartesianController::acceptGoal(boost::shared_ptr<const cob_cartesian_controller::CartesianControllerGoal> goal)
{
    trajectory_action ta;
    ta.name = goal->ta.name;

    if(ta.name == "move_lin")
    {
        ta.move_lin.x           = goal->ta.move_lin.x;
        ta.move_lin.y           = goal->ta.move_lin.y;
        ta.move_lin.z           = goal->ta.move_lin.z;
        ta.move_lin.roll        = goal->ta.move_lin.roll * M_PI/180;
        ta.move_lin.pitch       = goal->ta.move_lin.pitch * M_PI/180;
        ta.move_lin.yaw         = goal->ta.move_lin.yaw * M_PI/180;
        ta.move_lin.vel         = goal->ta.move_lin.vel;
        ta.move_lin.accl        = goal->ta.move_lin.accl;
        ta.move_lin.rotate_only = goal->ta.move_lin.rotateOnly;
        ta.move_lin.profile     = goal->ta.move_lin.profile;
    }
    else if(ta.name == "move_circ")
    {
        ta.move_circ.x_center      = goal->ta.move_circ.x_center;
        ta.move_circ.y_center      = goal->ta.move_circ.y_center;
        ta.move_circ.z_center      = goal->ta.move_circ.z_center;
        ta.move_circ.roll_center   = goal->ta.move_circ.roll_center * M_PI/180;
        ta.move_circ.pitch_center  = goal->ta.move_circ.pitch_center * M_PI/180;
        ta.move_circ.yaw_center    = goal->ta.move_circ.yaw_center * M_PI/180;
        ta.move_circ.startAngle    = goal->ta.move_circ.start_angle;
        ta.move_circ.endAngle      = goal->ta.move_circ.end_angle;
        ta.move_circ.radius        = goal->ta.move_circ.radius;
        ta.move_circ.vel           = goal->ta.move_circ.vel;
        ta.move_circ.accl          = goal->ta.move_circ.accl;
        ta.move_circ.profile       = goal->ta.move_circ.profile;
    }
    else if(ta.name == "hold")
    {
        ta.hold_time = goal -> ta.hold_time;
    }

    return ta;
}

trajectory_action_move_lin CartesianController::convertActionIntoMoveLin(trajectory_action ta)
{
    tf::TransformListener listener;
    geometry_msgs::Pose actualTcpPose, end;
    tf::Quaternion q_start,q_end, q_rel;

    actualTcpPose = utils_.getEndeffectorPose(listener, referenceFrame_, chain_tip_link_);

    // Transform RPY to Quaternion
    q_rel.setRPY(ta.move_lin.roll, ta.move_lin.pitch, ta.move_lin.yaw);

    q_start = tf::Quaternion(actualTcpPose.orientation.x,
                             actualTcpPose.orientation.y,
                             actualTcpPose.orientation.z,
                             actualTcpPose.orientation.w);

    q_end = q_start * q_rel;

    // Define End Pose
    end.position.x = actualTcpPose.position.x + ta.move_lin.x;
    end.position.y = actualTcpPose.position.y + ta.move_lin.y;
    end.position.z = actualTcpPose.position.z + ta.move_lin.z;
    end.orientation.x = q_end.getX();
    end.orientation.y = q_end.getY();
    end.orientation.z = q_end.getZ();
    end.orientation.w = q_end.getW();

    utils_.PoseToRPY(actualTcpPose, ta.move_lin.roll, ta.move_lin.pitch, ta.move_lin.yaw);
    ta.move_lin.start = actualTcpPose;
    ta.move_lin.end = end;

    return ta.move_lin;
}

trajectory_action_move_circ CartesianController::convertActionIntoMoveCirc(trajectory_action ta)
{
    return ta.move_circ;
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
