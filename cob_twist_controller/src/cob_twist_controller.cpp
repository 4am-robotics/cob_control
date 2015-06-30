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
 *   ROS package name: cob_twist_controller
 *
 * \author
 *   Author: Felix Messmer, email: Felix.Messmer@ipa.fraunhofer.de
 *
 * \date Date of creation: April, 2014
 *
 * \brief
 *   This package provides a generic Twist controller for the Care-O-bot
 *
 ****************************************************************/
#include <ros/ros.h>

#include <cob_twist_controller/cob_twist_controller.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>

#include "cob_obstacle_distance/Registration.h"

#define DEBUG_BASE_ACTIVE    0
#define DEBUG_BASE_COMP     0

bool CobTwistController::initialize()
{
    ros::NodeHandle nh_twist("twist_controller");

    // JointNames
    if(!nh_.getParam("joint_names", joints_))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }
    twist_controller_params_.dof = joints_.size();

    // Chain
    if(!nh_.getParam("chain_base_link", chain_base_link_))
    {
        ROS_ERROR("Parameter 'chain_base_link' not set");
        return false;
    }
    if (!nh_.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    // Multi-Chain Support

    // Cartesian VelLimits
    if (!nh_twist.getParam("max_vel_lin", twist_controller_params_.max_vel_lin))
    {
        twist_controller_params_.max_vel_lin = 10.0;    //m/sec
        ROS_WARN_STREAM("Parameter 'max_vel_lin' not set. Using default: " << twist_controller_params_.max_vel_lin);
    }
    if (!nh_twist.getParam("max_vel_rot", twist_controller_params_.max_vel_rot))
    {
        twist_controller_params_.max_vel_rot = 6.28;    //rad/sec
        ROS_WARN_STREAM("Parameter 'max_vel_rot' not set. Using default: " << twist_controller_params_.max_vel_rot);
    }
    if (!nh_twist.getParam("max_vel_lin_base", twist_controller_params_.max_vel_lin_base))
    {
        twist_controller_params_.max_vel_lin_base = 0.5;    //m/sec
        ROS_WARN_STREAM("Parameter 'max_vel_lin_base' not set. Using default: " << twist_controller_params_.max_vel_lin_base);
    }
    if (!nh_twist.getParam("max_vel_rot_base", twist_controller_params_.max_vel_rot_base))
    {
        twist_controller_params_.max_vel_rot_base = 0.5;    //rad/sec
        ROS_WARN_STREAM("Parameter 'max_vel_rot_base' not set. Using default: " << twist_controller_params_.max_vel_rot_base);
    }

    // Frames of Interest
    if(!nh_twist.getParam("collision_check_frames", twist_controller_params_.collision_check_frames))
    {
        ROS_ERROR_STREAM("Parameter vector 'collision_check_frames' not set.");
        return false;
    }

    ///parse robot_description and generate KDL chains
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromParam("/robot_description", my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    my_tree.getChain(chain_base_link_, chain_tip_link_, chain_);
    if(chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    ///parse robot_description and set velocity limits
    urdf::Model model;
    if (!model.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file for JointLimits");
        return false;
    }

    for(unsigned int i=0; i < twist_controller_params_.dof; i++)
    {
        twist_controller_params_.limits_vel.push_back(model.getJoint(joints_[i])->limits->velocity);
        twist_controller_params_.limits_min.push_back(model.getJoint(joints_[i])->limits->lower);
        twist_controller_params_.limits_max.push_back(model.getJoint(joints_[i])->limits->upper);
    }

    ///initialize configuration control solver
    p_inv_diff_kin_solver_.reset(new InverseDifferentialKinematicsSolver(chain_, callback_data_mediator_));

    // Before setting up dynamic_reconfigure server: init AugmentedSolverParams with default values
    this->initInvDiffKinSolverParams();

    ///Setting up dynamic_reconfigure server for the AugmentedSolverParams
    reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_twist_controller::TwistControllerConfig>(reconfig_mutex_, nh_twist));
    reconfigure_server_->setCallback(boost::bind(&CobTwistController::reconfigureCallback,   this, _1, _2));

    ///initialize variables and current joint values and velocities
    last_q_ = KDL::JntArray(chain_.getNrOfJoints());
    last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());

    ///give tf_listener some time to fill tf-cache
    ros::Duration(1.0).sleep();

    ///initialize ROS interfaces
    obstacle_distance_sub_ = nh_.subscribe("obstacle_distance", 1, &CallbackDataMediator::distancesToObstaclesCallback, &callback_data_mediator_);
    jointstate_sub = nh_.subscribe("joint_states", 1, &CobTwistController::jointstateCallback, this);
    twist_sub = nh_twist.subscribe("command_twist", 1, &CobTwistController::twistCallback, this);
    twist_stamped_sub = nh_twist.subscribe("command_twist_stamped", 1, &CobTwistController::twistStampedCallback, this);

//    vel_pub = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);
//    pos_pub = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_position_controller/command", 1);

    odometry_sub = nh_.subscribe("/base/odometry_controller/odometry", 1, &CobTwistController::odometryCallback, this);
    base_vel_pub = nh_.advertise<geometry_msgs::Twist>("/base/twist_controller/command", 1);


    /// Debug
    #if DEBUG_BASE_COMP == 1
        debug_base_compensation_visual_tip_pub_ = nh_twist.advertise<visualization_msgs::Marker>( "debug/vis_tip", 0 );
        debug_base_compensation_visual_base_pub_ = nh_twist.advertise<visualization_msgs::Marker>( "debug/vis_base", 0 );
        debug_base_compensation_pose_base_pub_ = nh_twist.advertise<geometry_msgs::Pose> ("debug/pose_base", 0);
        debug_base_compensation_pose_tip_pub_ = nh_twist.advertise<geometry_msgs::Pose> ("debug/pose_tip", 0);
        debug_base_compensation_twist_manipulator_pub_ = nh_twist.advertise<geometry_msgs::Twist> ("debug/twist_mani", 1);
    #endif

    #if DEBUG_BASE_ACTIVE == 1
        debug_base_active_twist_manipulator_pub_ = nh_twist.advertise<geometry_msgs::Twist> ("debug/twist_mani", 1);
        debug_base_active_twist_base_pub_ = nh_twist.advertise<geometry_msgs::Twist> ("debug/twist_base", 1);
        debug_base_active_twist_ee_pub_ = nh_twist.advertise<geometry_msgs::Twist> ("debug/twist_ee", 1);
    #endif

    ros::Time time_ = ros::Time::now();
    ros::Time last_update_time_ = time_;
    ros::Duration period_ = time_ - last_update_time_;

    this->twist_controller_params_.keep_direction = true;
    this->twist_controller_params_.enforce_pos_limits = true;
    this->twist_controller_params_.enforce_vel_limits = true;

    this->limiters_.reset(new LimiterContainer(this->twist_controller_params_, this->chain_));
    this->limiters_->init();

    this->interface_.reset(InterfaceBuilder::create_interface(this->nh_, this->twist_controller_params_));

    for(int i = 0; i < 6; i++)
    {
        ma_base_vel_smoother_.push_back(MovingAverage(10));
    }
    ROS_INFO("...initialized!");
    return true;
}

void CobTwistController::reinitServiceRegistration()
{
    ROS_INFO("Reinit of Service registration!");
    ros::ServiceClient client = nh_.serviceClient<cob_obstacle_distance::Registration>("obstacle_distance/registerPointOfInterest");
    ROS_INFO_STREAM("Created service client for service " << nh_.getNamespace() << "/obstacle_distance/registerPointOfInterest");

    for(std::vector<std::string>::const_iterator it = twist_controller_params_.collision_check_frames.begin();
            it != twist_controller_params_.collision_check_frames.end();
            it++)
    {
        cob_obstacle_distance::Registration r;
        r.request.frame_id = *it;
        r.request.shape_type = visualization_msgs::Marker::SPHERE;
        if (client.call(r))
        {
            ROS_INFO_STREAM("Called registration service with success: " << int(r.response.success) << ". Got message: " << r.response.message);
        }
        else
        {
            ROS_WARN_STREAM("Failed to call registration service for namespace: " << nh_.getNamespace());
            break;
        }
    }
}

void CobTwistController::reconfigureCallback(cob_twist_controller::TwistControllerConfig &config, uint32_t level)
{
    InvDiffKinSolverParams params;
    params.damping_method = static_cast<DampingMethodTypes>(config.damping_method);
    params.numerical_filtering = config.numerical_filtering;
    params.damping_factor = config.damping_factor;
    params.lambda_max = config.lambda_max;
    params.w_threshold = config.w_threshold;
    params.beta = config.beta;
    params.eps_damping = config.eps_damping;
    params.constraint = static_cast<ContraintTypes>(config.constraint);
    params.eps_truncation = config.eps_truncation;
    params.base_compensation = config.base_compensation;
    params.base_active = config.base_active;
    params.base_ratio = config.base_ratio;
    params.limits_min = twist_controller_params_.limits_min; // from cob_twist_controller init
    params.limits_max = twist_controller_params_.limits_max; // from cob_twist_controller init
    params.limits_vel = twist_controller_params_.limits_vel; // from cob_twist_controller init
    params.frame_names.clear();
    for (uint16_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        params.frame_names.push_back(chain_.getSegment(i).getName());
    }

    params.k_H = config.k_H;

    reset_markers_ = config.reset_markers;

    twist_controller_params_.interface_type = static_cast<InterfaceType>(config.interface_type);
    twist_controller_params_.enforce_pos_limits = config.enforce_pos_limits;
    twist_controller_params_.enforce_vel_limits = config.enforce_vel_limits;
    twist_controller_params_.base_active = config.base_active;
    twist_controller_params_.base_compensation = config.base_compensation;
    twist_controller_params_.tolerance = config.tolerance;
    twist_controller_params_.keep_direction = config.keep_direction;

    this->limiters_.reset(new LimiterContainer(this->twist_controller_params_, this->chain_));
    this->limiters_->init();

    this->interface_.reset(InterfaceBuilder::create_interface(this->nh_, this->twist_controller_params_));


    if(twist_controller_params_.base_active && twist_controller_params_.base_compensation)
    {
        ROS_ERROR("base_active and base_compensation cannot be enabled at the same time");
    }

    p_inv_diff_kin_solver_->SetInvDiffKinSolverParams(params);

    this->reinitServiceRegistration();
}

void CobTwistController::initInvDiffKinSolverParams()
{
    if(NULL == this->p_inv_diff_kin_solver_)
    {
        ROS_ERROR("p_inv_diff_kin_solver_ not yet initialized.");
        return;
    }

    InvDiffKinSolverParams params;
    params.damping_method = MANIPULABILITY;
    params.constraint = WLN_JLA;
    params.eps_truncation = 0.001;
    params.damping_factor = 0.2;
    params.lambda_max = 0.1;
    params.w_threshold = 0.005;
    params.base_compensation = false;
    params.base_active = false;
    params.base_ratio = 0.0;
    params.limits_min = twist_controller_params_.limits_min;
    params.limits_max = twist_controller_params_.limits_max;
    params.limits_vel = twist_controller_params_.limits_vel;
    params.k_H = 1.0;

    params.frame_names.clear();
    for (uint16_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        params.frame_names.push_back(chain_.getSegment(i).getName());
    }

    p_inv_diff_kin_solver_->SetInvDiffKinSolverParams(params);
}

void CobTwistController::run()
{
    ROS_INFO("cob_twist_controller...spinning");
    ros::spin();
}

/// Orientation of twist_stamped_msg is with respect to coordinate system given in header.frame_id
void CobTwistController::twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    tf::StampedTransform transform_tf;
    KDL::Frame frame;
    KDL::Twist twist, twist_transformed;

    try{
        tf_listener_.lookupTransform(chain_base_link_, msg->header.frame_id, ros::Time(0), transform_tf);
        frame.M = KDL::Rotation::Quaternion(transform_tf.getRotation().x(), transform_tf.getRotation().y(), transform_tf.getRotation().z(), transform_tf.getRotation().w());
    }
    catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        return;
    }

    tf::twistMsgToKDL(msg->twist, twist);
    twist_transformed = frame*twist;
    solveTwist(twist_transformed);
}

/// Orientation of twist_msg is with respect to chain_base coordinate system
void CobTwistController::twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    KDL::Twist twist;
    tf::twistMsgToKDL(*msg, twist);
    solveTwist(twist);
}

/// Orientation of twist is with respect to chain_base coordinate system
void CobTwistController::solveTwist(KDL::Twist twist)
{
    geometry_msgs::Pose pose_tip, pose_base;
    geometry_msgs::Point point_base, point_ee;

    int ret_ik;
    KDL::JntArray q_dot_ik(chain_.getNrOfJoints());

    if(twist_controller_params_.base_active)
    {
        #if DEBUG == 1
            debug();
            if(!reset_markers_){
                point_ee.x = odom_transform_ct.getOrigin().x();
                point_ee.y = odom_transform_ct.getOrigin().y();
                point_ee.z = odom_transform_ct.getOrigin().z();

                point_base.x = odom_transform_bl.getOrigin().x();
                point_base.y = odom_transform_bl.getOrigin().y();
                point_base.z = odom_transform_bl.getOrigin().z();

                point_ee_vec_.push_back(point_ee);
                point_base_vec_.push_back(point_base);

                double id1 = 0;
                double id2 = 1;

                showMarker(id1,1,0,0,"m",debug_base_compensation_visual_tip_pub_,point_ee_vec_);
                showMarker(id2,0,1,0,"m",debug_base_compensation_visual_base_pub_,point_base_vec_);
            }

            if(reset_markers_){
                point_ee_vec_.clear();
                point_base_vec_.clear();
            }
        #endif
        q_dot_ik.resize(chain_.getNrOfJoints() + 6); // +6 for max. additional DoF
        try
        {
            tf_listener_.waitForTransform("base_link",chain_tip_link_, ros::Time(0), ros::Duration(0.5));
            tf_listener_.lookupTransform("base_link",chain_tip_link_,  ros::Time(0), bl_transform_ct);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }

        bl_frame_ct.p = KDL::Vector(bl_transform_ct.getOrigin().x(), bl_transform_ct.getOrigin().y(), bl_transform_ct.getOrigin().z());
        bl_frame_ct.M = KDL::Rotation::Quaternion(bl_transform_ct.getRotation().x(), bl_transform_ct.getRotation().y(), bl_transform_ct.getRotation().z(), bl_transform_ct.getRotation().w());


        try
        {
            tf_listener_.waitForTransform(chain_base_link_,"base_link", ros::Time(0), ros::Duration(0.5));
            tf_listener_.lookupTransform(chain_base_link_,"base_link", ros::Time(0), cb_transform_bl);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }

        cb_frame_bl.p = KDL::Vector(cb_transform_bl.getOrigin().x(), cb_transform_bl.getOrigin().y(), cb_transform_bl.getOrigin().z());
        cb_frame_bl.M = KDL::Rotation::Quaternion(cb_transform_bl.getRotation().x(), cb_transform_bl.getRotation().y(), cb_transform_bl.getRotation().z(), cb_transform_bl.getRotation().w());

        //Solve twist
        // 1 for possible DoF of the extended system. In the Base_Active case it looks like the following configuration
        ExtendedJacobianDimension dim;
        dim.lin_x=1;
        dim.lin_y=1;
        dim.lin_z=0;

        dim.rot_x=0;
        dim.rot_y=0;
        dim.rot_z=1;
        ret_ik = p_inv_diff_kin_solver_->CartToJnt(last_q_, last_q_dot_, twist, bl_frame_ct, cb_frame_bl, q_dot_ik, dim);
    }

    if(twist_controller_params_.base_compensation)
    {
        twist = twist - twist_odometry_cb_;

        #if DEBUG_BASE_COMP == 1
            debug();
            tf::poseKDLToMsg(odom_frame_ct,pose_tip);
            tf::poseKDLToMsg(odom_frame_bl,pose_base);
            geometry_msgs::Twist twist_manipulator_bl;

            ////Twist Manipulator in base_link
            tf::twistKDLToMsg(bl_frame_cb*twist,twist_manipulator_bl);

            //Debug publisher
            debug_base_compensation_pose_base_pub_.publish(pose_base);
            debug_base_compensation_pose_tip_pub_.publish(pose_tip);
            debug_base_compensation_twist_manipulator_pub_.publish(twist_manipulator_bl);    // Base_link
        #endif
    }

    if(!twist_controller_params_.base_active)
    {
        ret_ik = p_inv_diff_kin_solver_->CartToJnt(last_q_, last_q_dot_, twist, q_dot_ik);
    }

    if(0 != ret_ik)
    {
        ROS_ERROR("No Vel-IK found!");
    }
    else
    {
        q_dot_ik = this->limiters_->enforceLimits(q_dot_ik, last_q_);

        // Change between velocity and position interface
        this->interface_->process_result(q_dot_ik, initial_pos_);


        if(twist_controller_params_.base_active)
        {
            geometry_msgs::Twist base_vel_msg;
            /// Base Velocities with respect to base_link
            base_vel_msg.linear.x = q_dot_ik(twist_controller_params_.dof);
            base_vel_msg.linear.y = q_dot_ik(twist_controller_params_.dof + 1);
            base_vel_msg.linear.z = q_dot_ik(twist_controller_params_.dof + 2);
            base_vel_msg.angular.x = q_dot_ik(twist_controller_params_.dof + 3);
            base_vel_msg.angular.y = q_dot_ik(twist_controller_params_.dof + 4);
            base_vel_msg.angular.z = q_dot_ik(twist_controller_params_.dof + 5);
            base_vel_pub.publish(base_vel_msg);

            #if DEBUG_BASE_ACTIVE == 1
                KDL::Twist twist_base_bl,twist_manipulator_cb,twist_manipulator_bl;
                KDL::FrameVel FrameVel_cb;

                geometry_msgs::Twist twist_manipulator_msg,twist_combined_msg;

                tf::twistMsgToKDL(base_vel_msg, twist_base_bl);
                debug_twistControllerParams_.base_activetwist_base_pub_.publish(base_vel_msg);    // Base twist in base_link

                /////calculate current Manipulator-Twists
                KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(last_q_,last_q_dot_);
                jntToCartSolver_vel_.reset(new KDL::ChainFkSolverVel_recursive(chain_));
                int ret = jntToCartSolver_vel_->JntToCart(jntArrayVel,FrameVel_cb,-1);

                if(ret>=0){
                    twist_manipulator_cb = FrameVel_cb.GetTwist();
                    twist_manipulator_bl = bl_frame_cb * twist_manipulator_cb;
                    tf::twistKDLToMsg(twist_manipulator_bl,twist_manipulator_msg);    // Manipulator twist in base_link
                }
                else{
                    ROS_WARN("ChainFkSolverVel failed!");
                }

                debug_twistControllerParams_.base_activetwist_manipulator_pub_.publish(twist_manipulator_msg);
                tf::twistKDLToMsg(twist_base_bl+twist_manipulator_bl,twist_combined_msg);    // Combined twist in base_link
                debug_twistControllerParams_.base_activetwist_ee_pub_.publish(twist_combined_msg);
            #endif
        }
    }
}


void CobTwistController::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = last_q_;
    KDL::JntArray q_dot_temp = last_q_dot_;
    int count = 0;

    for(unsigned int j = 0; j < twist_controller_params_.dof; j++)
    {
        for(unsigned int i = 0; i < msg->name.size(); i++)
        {
            if(strcmp(msg->name[i].c_str(), joints_[j].c_str()) == 0)
            {
                q_temp(j) = msg->position[i];
                q_dot_temp(j) = msg->velocity[i];
                count++;
                break;
            }
        }
    }

    if(count == joints_.size())
    {
        last_q_ = q_temp;
        last_q_dot_ = q_dot_temp;
    }
    initial_pos_.clear();

    for(int i = 0; i< msg->position.size();i++)
    {
        initial_pos_.push_back(msg->position[i]);
    }
}

void CobTwistController::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    KDL::Twist twist_odometry_bl, tangential_twist_bl, twist_odometry_transformed_cb;

    try{
        tf_listener_.waitForTransform(chain_base_link_,"base_link", ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform(chain_base_link_,"base_link", ros::Time(0), cb_transform_bl);

        tf_listener_.waitForTransform("base_link",chain_tip_link_, ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform("base_link",chain_tip_link_, ros::Time(0), bl_transform_ct);

        cb_frame_bl.p = KDL::Vector(cb_transform_bl.getOrigin().x(), cb_transform_bl.getOrigin().y(), cb_transform_bl.getOrigin().z());
        cb_frame_bl.M = KDL::Rotation::Quaternion(cb_transform_bl.getRotation().x(), cb_transform_bl.getRotation().y(), cb_transform_bl.getRotation().z(), cb_transform_bl.getRotation().w());
    }
    catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        return;
    }
    try
    {
        // Calculate tangential twist for angular base movements v = w x r
        Eigen::Vector3d r(bl_transform_ct.getOrigin().x(),bl_transform_ct.getOrigin().y(),bl_transform_ct.getOrigin().z());
        Eigen::Vector3d w(0,0,msg->twist.twist.angular.z);
        Eigen::Vector3d res = w.cross(r);
        tangential_twist_bl.vel = KDL::Vector(res(0),res(1),res(2));
        tangential_twist_bl.rot = KDL::Vector(0,0,0);
    }
    catch(...)
    {
        ROS_ERROR("Error occurred while calculating tangential twist for angular base movements.");
        return;
    }

    //ROS_INFO("Crossproduct : %f %f %f",res(0),res(1),res(2));
    //ROS_INFO("TCP: %f %f %f",bl_transform_ct.getOrigin().x(),bl_transform_ct.getOrigin().y(),bl_transform_ct.getOrigin().z());

    tf::twistMsgToKDL(msg->twist.twist, twist_odometry_bl);    // Base Twist

    // transform into chain_base
    twist_odometry_transformed_cb = cb_frame_bl * (twist_odometry_bl+tangential_twist_bl);

    twist_odometry_cb_ = twist_odometry_transformed_cb;
}


void CobTwistController::showMarker(int marker_id,double red, double green, double blue, std::string ns, ros::Publisher pub, std::vector<geometry_msgs::Point> &pos_v)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom_combined";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;

    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;

    marker.points.insert(marker.points.begin(), pos_v.begin(), pos_v.end());

    marker.color.a = 1.0;
    pub.publish( marker );
}

void CobTwistController::debug()
{
    try{
        tf_listener_.waitForTransform("base_link",chain_base_link_, ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform("base_link",chain_base_link_, ros::Time(0), bl_transform_cb);
    }catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        return;
    }

    try{
        tf_listener_.waitForTransform("odom_combined",chain_tip_link_, ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform("odom_combined",chain_tip_link_, ros::Time(0), odom_transform_ct);
    }catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        return;
    }
    try{
        tf_listener_.waitForTransform("odom_combined","base_link", ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform("odom_combined","base_link", ros::Time(0), odom_transform_bl);
    }catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        return;
    }
    odom_frame_ct.p = KDL::Vector(odom_transform_ct.getOrigin().x(), odom_transform_ct.getOrigin().y(), odom_transform_ct.getOrigin().z());
    odom_frame_ct.M = KDL::Rotation::Quaternion(odom_transform_ct.getRotation().x(), odom_transform_ct.getRotation().y(), odom_transform_ct.getRotation().z(), odom_transform_ct.getRotation().w());

    odom_frame_bl.p = KDL::Vector(odom_transform_bl.getOrigin().x(), odom_transform_bl.getOrigin().y(), odom_transform_bl.getOrigin().z());
    odom_frame_bl.M = KDL::Rotation::Quaternion(odom_transform_bl.getRotation().x(), odom_transform_bl.getRotation().y(), odom_transform_bl.getRotation().z(), odom_transform_bl.getRotation().w());

    bl_frame_cb.p = KDL::Vector(bl_transform_cb.getOrigin().x(), bl_transform_cb.getOrigin().y(), bl_transform_cb.getOrigin().z());
    bl_frame_cb.M = KDL::Rotation::Quaternion(bl_transform_cb.getRotation().x(), bl_transform_cb.getRotation().y(), bl_transform_cb.getRotation().z(), bl_transform_cb.getRotation().w());
}

