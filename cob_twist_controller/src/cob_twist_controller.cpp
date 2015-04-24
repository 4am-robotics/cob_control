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

#define DEBUG_BASE_ACTIVE    0
#define DEBUG_BASE_COMP     0

bool CobTwistController::initialize()
{
    ros::NodeHandle nh_twist("twist_controller");
    ros::NodeHandle nh_cartesian("cartesian_controller");

    // JointNames
    if(!nh_.getParam("joint_names", joints_))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }
    twistControllerParams_.dof = joints_.size();

    // Chain
    if(!nh_cartesian.getParam("chain_base_link", chain_base_link_))
    {
        ROS_ERROR("Parameter 'chain_base_link' not set");
        return false;
    }
    if (!nh_cartesian.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }


    // Multi-Chain Support

    // Cartesian VelLimits
    if (!nh_cartesian.getParam("max_vel_lin", twistControllerParams_.max_vel_lin))
    {
        twistControllerParams_.max_vel_lin = 10.0;    //m/sec
        ROS_WARN_STREAM("Parameter 'max_vel_lin' not set. Using default: " << twistControllerParams_.max_vel_lin);
    }
    if (!nh_cartesian.getParam("max_vel_rot", twistControllerParams_.max_vel_rot))
    {
        twistControllerParams_.max_vel_rot = 6.28;    //rad/sec
        ROS_WARN_STREAM("Parameter 'max_vel_rot' not set. Using default: " << twistControllerParams_.max_vel_rot);
    }
    if (!nh_cartesian.getParam("max_vel_lin_base", twistControllerParams_.max_vel_lin_base))
    {
        twistControllerParams_.max_vel_lin_base = 0.5;    //m/sec
        ROS_WARN_STREAM("Parameter 'max_vel_lin_base' not set. Using default: " << twistControllerParams_.max_vel_lin_base);
    }
    if (!nh_cartesian.getParam("max_vel_rot_base", twistControllerParams_.max_vel_rot_base))
    {
        twistControllerParams_.max_vel_rot_base = 0.5;    //rad/sec
        ROS_WARN_STREAM("Parameter 'max_vel_rot_base' not set. Using default: " << twistControllerParams_.max_vel_rot_base);
    }

    ///parse robot_description and generate KDL chains
    KDL::Tree my_tree;
    std::string robot_desc_string;
    nh_.param("/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
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

    for(unsigned int i=0; i < twistControllerParams_.dof; i++)
    {
        twistControllerParams_.limits_vel.push_back(model.getJoint(joints_[i])->limits->velocity);
        twistControllerParams_.limits_min.push_back(model.getJoint(joints_[i])->limits->lower);
        twistControllerParams_.limits_max.push_back(model.getJoint(joints_[i])->limits->upper);
    }

    ///initialize configuration control solver
    p_augmented_solver_.reset(new AugmentedSolver(chain_, 0.001));

    // Before setting up dynamic_reconfigure server: init AugmentedSolverParams with default values
    this->initAugmentedSolverParams();

    ///Setting up dynamic_reconfigure server for the AugmentedSolverParams
    reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_twist_controller::TwistControllerConfig>(reconfig_mutex_, nh_cartesian));
    reconfigure_server_->setCallback(boost::bind(&CobTwistController::reconfigure_callback,   this, _1, _2));

    ///initialize variables and current joint values and velocities
    last_q_ = KDL::JntArray(chain_.getNrOfJoints());
    last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());

    ///give tf_listener some time to fill tf-cache
    ros::Duration(1.0).sleep();

    ///initialize ROS interfaces
    jointstate_sub = nh_.subscribe("joint_states", 1, &CobTwistController::jointstate_cb, this);
    twist_sub = nh_twist.subscribe("command_twist", 1, &CobTwistController::twist_cb, this);
    twist_stamped_sub = nh_twist.subscribe("command_twist_stamped", 1, &CobTwistController::twist_stamped_cb, this);
    vel_pub = nh_.advertise<std_msgs::Float64MultiArray>("joint_group_velocity_controller/command", 1);

    odometry_sub = nh_.subscribe("/base/odometry_controller/odometry", 1, &CobTwistController::odometry_cb, this);
    base_vel_pub = nh_.advertise<geometry_msgs::Twist>("/base/twist_controller/command", 1);


    /// Debug
    twist_current_pub_ = nh_twist.advertise<geometry_msgs::Twist> ("debug/twist_current", 1);
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

    this->limiters_.reset(new LimiterContainer(this->twistControllerParams_, this->chain_));
    this->limiters_->init();

    ROS_INFO("...initialized!");
    return true;
}

void CobTwistController::reconfigure_callback(cob_twist_controller::TwistControllerConfig &config, uint32_t level)
{
    AugmentedSolverParams params;
    params.damping_method = static_cast<DampingMethodTypes>(config.damping_method);
    params.constraint = static_cast<ContraintTypes>(config.constraint);
    params.eps = config.eps;
    params.damping_factor = config.damping_factor;
    params.lambda0 = config.lambda0;
    params.wt = config.wt;
    params.base_compensation = config.base_compensation;
    params.base_active = config.base_active;
    params.base_ratio = config.base_ratio;

    params.limits_min = twistControllerParams_.limits_min; // from cob_twist_controller init
    params.limits_max = twistControllerParams_.limits_max; // from cob_twist_controller init

    reset_markers_ = config.reset_markers;


    twistControllerParams_.enforce_pos_limits = config.enforce_pos_limits;
    twistControllerParams_.enforce_vel_limits = config.enforce_vel_limits;
    twistControllerParams_.base_active = config.base_active;
    twistControllerParams_.base_compensation = config.base_compensation;
    twistControllerParams_.tolerance = config.tolerance;
    twistControllerParams_.keep_direction = config.keep_direction;

    this->limiters_.reset(new LimiterContainer(this->twistControllerParams_, this->chain_));
    this->limiters_->init();

    if(twistControllerParams_.base_active && twistControllerParams_.base_compensation)
    {
        ROS_ERROR("base_active and base_compensation cannot be enabled at the same time");
    }

    p_augmented_solver_->SetAugmentedSolverParams(params);
}

void CobTwistController::initAugmentedSolverParams()
{
    if(NULL == this->p_augmented_solver_)
    {
        ROS_ERROR("p_augmented_solver_ not yet initialized.");
        return;
    }

    AugmentedSolverParams params;
    params.damping_method = MANIPULABILITY;
    params.constraint = WLN_JLA;
    params.eps = 0.001;
    params.damping_factor = 0.2;
    params.lambda0 = 0.1;
    params.wt = 0.005;
    params.base_compensation = false;
    params.base_active = false;
    params.base_ratio = 0.0;
    params.limits_min = twistControllerParams_.limits_min;
    params.limits_max = twistControllerParams_.limits_max;

    p_augmented_solver_->SetAugmentedSolverParams(params);
}



void CobTwistController::run()
{
    ROS_INFO("cob_twist_controller...spinning");
    ros::spin();
}


/// Orientation of twist_stamped_msg is with respect to coordinate system given in header.frame_id
void CobTwistController::twist_stamped_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
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

    //ROS_DEBUG("Twist Vel (%f, %f, %f)", twist.vel.x(), twist.vel.y(), twist.vel.z());
    //ROS_DEBUG("Twist Rot (%f, %f, %f)", twist.rot.x(), twist.rot.y(), twist.rot.z());
    //ROS_DEBUG("TwistTransformed Vel (%f, %f, %f)", twist_transformed.vel.x(), twist_transformed.vel.y(), twist_transformed.vel.z());
    //ROS_DEBUG("TwistTransformed Rot (%f, %f, %f)", twist_transformed.rot.x(), twist_transformed.rot.y(), twist_transformed.rot.z());

    solve_twist(twist_transformed);
}

/// Orientation of twist_msg is with respect to chain_base coordinate system
void CobTwistController::twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    KDL::Twist twist;
    tf::twistMsgToKDL(*msg, twist);

    solve_twist(twist);
}


/// Orientation of twist is with respect to chain_base coordinate system
void CobTwistController::solve_twist(KDL::Twist twist)
{
    geometry_msgs::Pose pose_tip, pose_base;
    geometry_msgs::Point point_base, point_ee;

    int ret_ik;
    KDL::JntArray q_dot_ik(chain_.getNrOfJoints());

    if(twistControllerParams_.base_active)
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
        q_dot_ik.resize(chain_.getNrOfJoints() + 3); // + 3 for base
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
        ret_ik = p_augmented_solver_->CartToJnt(last_q_, last_q_dot_, twist, bl_frame_ct, cb_frame_bl, q_dot_ik);
    }

    if(twistControllerParams_.base_compensation)
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


    if(!twistControllerParams_.base_active)
    {
        ret_ik = p_augmented_solver_->CartToJnt(last_q_, last_q_dot_, twist, q_dot_ik);
    }

    if(0 != ret_ik)
    {
        ROS_ERROR("No Vel-IK found!");
    }
    else
    {
        q_dot_ik = this->limiters_->enforceLimits(q_dot_ik, last_q_);

        std_msgs::Float64MultiArray vel_msg;
        for(unsigned int i=0; i < twistControllerParams_.dof; i++)
        {
            vel_msg.data.push_back(q_dot_ik(i));
            //ROS_DEBUG("DesiredVel %d: %f", i, q_dot_ik(i));
        }

        if(twistControllerParams_.base_active)
        {
            geometry_msgs::Twist base_vel_msg;
            /// Base Velocities with respect to base_link
            base_vel_msg.linear.x = q_dot_ik(twistControllerParams_.dof);
            base_vel_msg.linear.y = q_dot_ik(twistControllerParams_.dof + 1);
            base_vel_msg.angular.z = q_dot_ik(twistControllerParams_.dof + 2);

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

        vel_pub.publish(vel_msg);
    }
}


void CobTwistController::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = last_q_;
    KDL::JntArray q_dot_temp = last_q_dot_;
    int count = 0;

    for(unsigned int j = 0; j < twistControllerParams_.dof; j++)
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
}

void CobTwistController::odometry_cb(const nav_msgs::Odometry::ConstPtr& msg)
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

