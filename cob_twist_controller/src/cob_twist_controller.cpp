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


//ToDo: Should we re-add DEBUG_BASE_COMPENSATION stuff?

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
    if(!nh_.getParam("chain_base_link", twist_controller_params_.chain_base_link))
    {
        ROS_ERROR("Parameter 'chain_base_link' not set");
        return false;
    }
    if (!nh_.getParam("chain_tip_link", twist_controller_params_.chain_tip_link))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

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

    my_tree.getChain(twist_controller_params_.chain_base_link, twist_controller_params_.chain_tip_link, chain_);
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

    // Before setting up dynamic_reconfigure server: initParams with default values
    this->initParams();

    ///Setting up dynamic_reconfigure server for the TwistControlerConfig parameters
    reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_twist_controller::TwistControllerConfig>(reconfig_mutex_, nh_twist));
    reconfigure_server_->setCallback(boost::bind(&CobTwistController::reconfigureCallback,   this, _1, _2));

    ///initialize variables and current joint values and velocities
    this->joint_states_.current_q_ = KDL::JntArray(chain_.getNrOfJoints());
    this->joint_states_.current_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());
    this->joint_states_.last_q_ = KDL::JntArray(chain_.getNrOfJoints());
    this->joint_states_.last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());

    ///give tf_listener some time to fill tf-cache
    ros::Duration(1.0).sleep();

    ///initialize ROS interfaces
    obstacle_distance_sub_ = nh_.subscribe("obstacle_distance", 1, &CallbackDataMediator::distancesToObstaclesCallback, &callback_data_mediator_);
    jointstate_sub_ = nh_.subscribe("joint_states", 1, &CobTwistController::jointstateCallback, this);
    twist_sub_ = nh_twist.subscribe("command_twist", 1, &CobTwistController::twistCallback, this);
    twist_stamped_sub_ = nh_twist.subscribe("command_twist_stamped", 1, &CobTwistController::twistStampedCallback, this);

    odometry_sub_ = nh_.subscribe("/base/odometry_controller/odometry", 1, &CobTwistController::odometryCallback, this);

    this->twist_controller_params_.keep_direction = true;
    this->twist_controller_params_.enforce_pos_limits = true;
    this->twist_controller_params_.enforce_vel_limits = true;

    this->hardware_interface_.reset(HardwareInterfaceBuilder::createHardwareInterface(this->nh_, this->twist_controller_params_));

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

void CobTwistController::reconfigureCallback(cob_twist_controller::TwistControllerConfig& config, uint32_t level)
{
    if((config.kinematic_extension == BASE_ACTIVE) && config.base_compensation)
    {
        ROS_ERROR("base_active and base_compensation cannot be enabled at the same time");
    }
    
    
    twist_controller_params_.hardware_interface_type = static_cast<HardwareInterfaceTypes>(config.hardware_interface_type);
    
    twist_controller_params_.numerical_filtering = config.numerical_filtering;
    twist_controller_params_.damping_method = static_cast<DampingMethodTypes>(config.damping_method);
    twist_controller_params_.damping_factor = config.damping_factor;
    twist_controller_params_.lambda_max = config.lambda_max;
    twist_controller_params_.w_threshold = config.w_threshold;
    twist_controller_params_.beta = config.beta;
    twist_controller_params_.eps_damping = config.eps_damping;
    
    twist_controller_params_.constraint = static_cast<ContraintTypes>(config.constraint);
    twist_controller_params_.mu = config.mu;
    twist_controller_params_.k_H = config.k_H;
    
    twist_controller_params_.eps_truncation = config.eps_truncation;
    
    twist_controller_params_.keep_direction = config.keep_direction;
    twist_controller_params_.enforce_pos_limits = config.enforce_pos_limits;
    twist_controller_params_.enforce_vel_limits = config.enforce_vel_limits;
    twist_controller_params_.tolerance = config.tolerance;
    
    twist_controller_params_.base_compensation = config.base_compensation;
    twist_controller_params_.kinematic_extension = static_cast<KinematicExtensionTypes>(config.kinematic_extension);
    twist_controller_params_.base_ratio = config.base_ratio;
    

    this->hardware_interface_.reset(HardwareInterfaceBuilder::createHardwareInterface(this->nh_, this->twist_controller_params_));

    p_inv_diff_kin_solver_->resetAll(this->twist_controller_params_);

    this->reinitServiceRegistration();
}

void CobTwistController::initParams()
{
    if(NULL == this->p_inv_diff_kin_solver_)
    {
        ROS_ERROR("p_inv_diff_kin_solver_ not yet initialized.");
        return;
    }

    TwistControllerParams params;
    
    params.dof = twist_controller_params_.dof;
    params.chain_base_link = twist_controller_params_.chain_base_link;
    params.chain_tip_link = twist_controller_params_.chain_tip_link;
    params.hardware_interface_type = VELOCITY_INTERFACE;
    
    params.numerical_filtering = false;
    params.damping_method = MANIPULABILITY;
    params.damping_factor = 0.2;
    params.lambda_max = 0.1;
    params.w_threshold = 0.005;
    params.beta = 0.005;
    params.eps_damping = 0.003;
    
    params.constraint = WLN_JLA;
    params.mu = -2.0;
    params.k_H = 1.0;
    
    params.eps_truncation = 0.001;
    
    params.keep_direction = true;
    params.enforce_pos_limits = true;
    params.enforce_vel_limits = true;
    params.tolerance = 5.0;
    
    // added limits from URDF file
    params.limits_max = twist_controller_params_.limits_max;
    params.limits_min = twist_controller_params_.limits_min;
    params.limits_vel = twist_controller_params_.limits_vel;
    
    params.max_vel_lin = twist_controller_params_.max_vel_lin;
    params.max_vel_rot = twist_controller_params_.max_vel_rot;
    params.max_vel_lin_base = twist_controller_params_.max_vel_lin_base;
    params.max_vel_rot_base = twist_controller_params_.max_vel_rot_base;
    
    params.base_compensation = false;
    params.kinematic_extension = NO_EXTENSION;
    params.base_ratio = 0.0;

    params.frame_names.clear();
    for (uint16_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        params.frame_names.push_back(chain_.getSegment(i).getName());
    }
    
    // TODO: initialization
    //params.collision_check_frames.clear();
    //params.task_stack_controller = NULL;
    
    
    twist_controller_params_ = params;
    p_inv_diff_kin_solver_->resetAll(params);
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
        tf_listener_.lookupTransform(twist_controller_params_.chain_base_link, msg->header.frame_id, ros::Time(0), transform_tf);
        frame.M = KDL::Rotation::Quaternion(transform_tf.getRotation().x(), transform_tf.getRotation().y(), transform_tf.getRotation().z(), transform_tf.getRotation().w());
    }
    catch (tf::TransformException& ex){
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
    int ret_ik;
    KDL::JntArray q_dot_ik(chain_.getNrOfJoints());

    if(twist_controller_params_.base_compensation)
    {
        twist = twist - twist_odometry_cb_;
    }
    
    ret_ik = p_inv_diff_kin_solver_->CartToJnt(this->joint_states_,
                                               twist,
                                               q_dot_ik);

    if(0 != ret_ik)
    {
        ROS_ERROR("No Vel-IK found!");
    }
    else
    {
        // Change between velocity and position interface
        this->hardware_interface_->processResult(q_dot_ik, this->joint_states_.current_q_);
    }
}


void CobTwistController::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = this->joint_states_.current_q_;
    KDL::JntArray q_dot_temp = this->joint_states_.current_q_dot_;
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
        this->joint_states_.last_q_ = joint_states_.current_q_;
        this->joint_states_.last_q_dot_ = joint_states_.current_q_dot_;
        this->joint_states_.current_q_ = q_temp;
        this->joint_states_.current_q_dot_ = q_dot_temp;
    }
}

void CobTwistController::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    KDL::Twist twist_odometry_bl, tangential_twist_bl, twist_odometry_transformed_cb;
    KDL::Frame cb_frame_bl;
    tf::StampedTransform cb_transform_bl, bl_transform_ct;

    try{
        tf_listener_.waitForTransform(twist_controller_params_.chain_base_link, "base_link", ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform(twist_controller_params_.chain_base_link, "base_link", ros::Time(0), cb_transform_bl);

        tf_listener_.waitForTransform("base_link", twist_controller_params_.chain_tip_link, ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform("base_link", twist_controller_params_.chain_tip_link, ros::Time(0), bl_transform_ct);

        cb_frame_bl.p = KDL::Vector(cb_transform_bl.getOrigin().x(), cb_transform_bl.getOrigin().y(), cb_transform_bl.getOrigin().z());
        cb_frame_bl.M = KDL::Rotation::Quaternion(cb_transform_bl.getRotation().x(), cb_transform_bl.getRotation().y(), cb_transform_bl.getRotation().z(), cb_transform_bl.getRotation().w());
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }
    
    try
    {
        // Calculate tangential twist for angular base movements v = w x r
        Eigen::Vector3d r(bl_transform_ct.getOrigin().x(), bl_transform_ct.getOrigin().y(), bl_transform_ct.getOrigin().z());
        Eigen::Vector3d w(0,0,msg->twist.twist.angular.z);
        Eigen::Vector3d res = w.cross(r);
        tangential_twist_bl.vel = KDL::Vector(res(0), res(1), res(2));
        tangential_twist_bl.rot = KDL::Vector(0,0,0);
    }
    catch(...)
    {
        ROS_ERROR("Error occurred while calculating tangential twist for angular base movements.");
        return;
    }

    //ROS_INFO("Crossproduct : %f %f %f", res(0), res(1), res(2));
    //ROS_INFO("TCP: %f %f %f", bl_transform_ct.getOrigin().x(), bl_transform_ct.getOrigin().y(), bl_transform_ct.getOrigin().z());

    tf::twistMsgToKDL(msg->twist.twist, twist_odometry_bl);    // Base Twist

    // transform into chain_base
    twist_odometry_transformed_cb = cb_frame_bl * (twist_odometry_bl + tangential_twist_bl);

    twist_odometry_cb_ = twist_odometry_transformed_cb;
}
