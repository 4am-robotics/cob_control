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
        target_frame_ = "cartesian_target";
    }


    ROS_WARN("Waiting for Services...");
    start_tracking_ = nh_.serviceClient<cob_srvs::SetString>("frame_tracker/start_tracking");
    stop_tracking_ = nh_.serviceClient<std_srvs::Empty>("frame_tracker/stop_tracking");
    start_tracking_.waitForExistence();
    stop_tracking_.waitForExistence();
    tracking_ = false;

    trajectory_interpolator_.reset(new TrajectoryInterpolator(update_rate_));

    action_name_ = "cartesian_trajectory_action";
    as_.reset(new SAS_CartesianControllerAction_t(nh_, action_name_, false));
    as_->registerGoalCallback(boost::bind(&CartesianController::goalCB, this));
    as_->registerPreemptCallback(boost::bind(&CartesianController::preemptCB, this));
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
bool CartesianController::movePTP(geometry_msgs::Pose target_pose, double epsilon)
{
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
        stamped_transform = utils_.getStampedTransform(target_frame_, chain_tip_link_);

        // Wait for chain_tip_link to be within epsilon area of target_frame
        if(utils_.inEpsilonArea(stamped_transform, epsilon))
        {
            reached_pos_counter++;    // Count up if end effector position is in the epsilon area to avoid wrong values
        }

        if(reached_pos_counter >= 2*update_rate_) //has been close enough to target for 2 seconds
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
bool CartesianController::posePathBroadcaster(std::vector<geometry_msgs::Pose>& pose_vector)
{
    bool success = false;
    double epsilon = 0.1;
    int failure_counter = 0;

    ros::Rate rate(update_rate_);
    tf::StampedTransform stamped_transform;
    stamped_transform.frame_id_ = root_frame_;
    stamped_transform.child_frame_id_ = target_frame_;

    for(int i = 0; i < pose_vector.size()-1; i++)
    {
        if(!as_->isActive())
        {
            success = false;
            break;
        }
        
        // Send/Refresh target Frame
        stamped_transform.setOrigin( tf::Vector3(pose_vector.at(i).position.x,
                                                 pose_vector.at(i).position.y,
                                                 pose_vector.at(i).position.z) );
        stamped_transform.setRotation( tf::Quaternion(pose_vector.at(i).orientation.x,
                                                      pose_vector.at(i).orientation.y,
                                                      pose_vector.at(i).orientation.z,
                                                      pose_vector.at(i).orientation.w) );
        stamped_transform.stamp_ = ros::Time::now();
        tf_broadcaster_.sendTransform(stamped_transform);

        // Get transformation
        stamped_transform = utils_.getStampedTransform(target_frame_, chain_tip_link_);

        // Check whether chain_tip_link is within epsilon area of target_frame
        if(!utils_.inEpsilonArea(stamped_transform, epsilon))
        {
            failure_counter++;
        }
        else
        {
            if(failure_counter > 0)
            {
                failure_counter--;
            }
        }

        //ToDo: This functionality is already implemented in the frame_tracker
        //Use the the ActionInterface of the FrameTracker instead of the ServiceCalls and modify constraints in FrameTracker accordingly
        if(failure_counter > 100)
        {
            ROS_ERROR("Endeffector failed to track target_frame!");
            success = false;
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return success;
}


void CartesianController::goalCB()
{
    std::vector <geometry_msgs::Pose> pos_vec;
    cob_cartesian_controller::CartesianActionStruct action_struct;

    ROS_INFO("Received a new goal");

    action_struct = acceptGoal(as_->acceptNewGoal());

    if(action_struct.name == "move_lin")
    {
        ROS_INFO("move_lin");

        cob_cartesian_controller::MoveLinStruct move_lin = convertMoveLinRelToAbs(action_struct.move_lin);

        // Interpolate path
        if(!trajectory_interpolator_->linearInterpolation(pos_vec, move_lin))
        {
            actionAbort(false, "Failed to do interpolation for 'move_lin'");
            return;
        }
        
        //ToDo: maybe publish a "preview" of the path as MarkerArray?
        

        // Activate Tracking
        if(!startTracking())
        {
            actionAbort(false, "Failed to start traccking");
            return;
        }
        
        // Execute path
        if(!posePathBroadcaster(pos_vec))
        {
            actionAbort(false, "Failed to execute path for 'move_lin'");
            return;
        }
        
        // De-Activate Tracking
        if(!stopTracking())
        {
            actionAbort(false, "Failed to stop traccking");
            return;
        }
        
        actionSuccess(true, "move_lin succeeded!");
    }
    else if(action_struct.name == "move_circ")
    {
        ROS_INFO("move_circ");
        cob_cartesian_controller::MoveCircStruct move_circ = convertMoveCircRelToAbs(action_struct.move_circ);

        if(!trajectory_interpolator_->circularInterpolation(pos_vec, move_circ))
        {
            actionAbort(false, "Failed to do interpolation for 'move_circ'");
            return;
        }
        
        //ToDo: maybe publish a "preview" of the path as MarkerArray?
        

        // Activate Tracking
        if(!startTracking())
        {
            actionAbort(false, "Failed to start traccking");
            return;
        }
        
        // Move to start
        if(!movePTP(pos_vec.at(0), 0.03))
        {
            actionAbort(false, "Failed to movePTP to start");
            return;
        }
        
        // Execute path
        if(!posePathBroadcaster(pos_vec))
        {
            actionAbort(false, "Failed to execute path for 'move_circ'");
            return;
        }
        
        // De-Activate Tracking
        if(!stopTracking())
        {
            actionAbort(false, "Failed to stop traccking");
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

cob_cartesian_controller::MoveLinStruct CartesianController::convertMoveLinRelToAbs(const cob_cartesian_controller::MoveLinStruct& rel_move_lin)
{
    //ToDo: Use proper call to transformPose() here after the action description has been changed to be using a PoseStamped
    
    geometry_msgs::Pose start, end;
    tf::Quaternion q_start, q_end, q_rel;
    start = utils_.getPose(root_frame_, chain_tip_link_);   //current tcp pose
    
    tf::quaternionMsgToTF(start.orientation, q_start);
    q_rel.setRPY(rel_move_lin.roll, rel_move_lin.pitch, rel_move_lin.yaw);
    // q_end = q_start * q_rel; // this does a rotation with respect to the endeffector but the rotation should be relative to the reference frame
    q_end = q_rel * q_start; // this does a rotation with respect to the reference as well as the lin movement is done in abs. coordinates

    end.position.x = start.position.x + rel_move_lin.x;
    end.position.y = start.position.y + rel_move_lin.y;
    end.position.z = start.position.z + rel_move_lin.z;
    tf::quaternionTFToMsg(q_end, end.orientation);

    cob_cartesian_controller::MoveLinStruct update_ml = rel_move_lin;
    //ToDo: Why is roll, pitch and yaw set to respective values of start? What about x, y, z? Should it not be the goal?
    utils_.poseToRPY(start, update_ml.roll, update_ml.pitch, update_ml.yaw);
    update_ml.start = start;
    update_ml.end = end;

    return update_ml;
}


cob_cartesian_controller::MoveCircStruct CartesianController::convertMoveCircRelToAbs(cob_cartesian_controller::MoveCircStruct& rel_move_circ)
{
    return rel_move_circ;
}

cob_cartesian_controller::CartesianActionStruct CartesianController::acceptGoal(boost::shared_ptr<const cob_cartesian_controller::CartesianControllerGoal> goal)
{
    cob_cartesian_controller::CartesianActionStruct action_struct;
    action_struct.name = goal->name;
    
    //ToDo: use a geometry_msgs::PoseStamped with proper frame_id instead of (x, y, z, roll, pitch, yaw)!

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
        action_struct.move_circ.start_angle   = goal->move_circ.start_angle * M_PI / 180.0;
        action_struct.move_circ.end_angle     = goal->move_circ.end_angle  * M_PI / 180.0;
        action_struct.move_circ.radius        = goal->move_circ.radius;
        action_struct.move_circ.profile.vel           = goal->move_circ.profile.vel;
        action_struct.move_circ.profile.accl          = goal->move_circ.profile.accl;
        action_struct.move_circ.profile.profile_type  = goal->move_circ.profile.profile_type;
    }
    else
    {
        actionAbort(false, "Unknown trajectory action" + action_struct.name);
    }

    return action_struct;
}

void CartesianController::preemptCB()
{
    // De-Activate Tracking
    stopTracking();
    actionPreempt(true, "action preempted!");
}

void CartesianController::actionSuccess(bool success, std::string message)
{
    ROS_INFO_STREAM("Goal succeeded: " << message);
    action_result_.success = success;
    action_result_.message = message;
    as_->setSucceeded(action_result_, action_result_.message);
}

void CartesianController::actionPreempt(bool success, std::string message)
{
    ROS_WARN_STREAM("Goal preempted: " << message);
    action_result_.success = success;
    action_result_.message = message;
    as_->setPreempted(action_result_, action_result_.message);
}

void CartesianController::actionAbort(bool success, std::string message)
{
    ROS_ERROR_STREAM("Goal aborted: "  << message);
    action_result_.success = success;
    action_result_.message = message;
    as_->setAborted(action_result_, action_result_.message);
}
