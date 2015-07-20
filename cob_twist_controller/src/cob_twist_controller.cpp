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
#include "cob_srvs/SetString.h"

bool CobTwistController::initialize()
{
    ros::NodeHandle nh_twist("twist_controller");

    // JointNames
    if(!nh_.getParam("joint_names", twist_controller_params_.joints))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }

    twist_controller_params_.dof = twist_controller_params_.joints.size();

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

    // Frames of Interest
    if(!nh_twist.getParam("collision_check_frames", twist_controller_params_.collision_check_frames))
    {
        ROS_WARN_STREAM("Parameter vector 'collision_check_frames' not set. Collision Avoidance constraint will not do anything.");
        twist_controller_params_.collision_check_frames.clear();
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
        twist_controller_params_.limits_vel.push_back(model.getJoint(twist_controller_params_.joints[i])->limits->velocity);
        twist_controller_params_.limits_min.push_back(model.getJoint(twist_controller_params_.joints[i])->limits->lower);
        twist_controller_params_.limits_max.push_back(model.getJoint(twist_controller_params_.joints[i])->limits->upper);
    }

    twist_controller_params_.frame_names.clear();
    for (uint16_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        twist_controller_params_.frame_names.push_back(chain_.getSegment(i).getName());
    }

    ///initialize configuration control solver
    p_inv_diff_kin_solver_.reset(new InverseDifferentialKinematicsSolver(twist_controller_params_, chain_, callback_data_mediator_));
    p_inv_diff_kin_solver_->resetAll(twist_controller_params_);

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

    this->hardware_interface_.reset(HardwareInterfaceBuilder::createHardwareInterface(this->nh_, this->twist_controller_params_));

    ROS_INFO("...initialized!");
    return true;
}

void CobTwistController::reinitServiceRegistration()
{
    ROS_INFO("Reinit of Service registration!");
    ros::ServiceClient client = nh_.serviceClient<cob_srvs::SetString>("obstacle_distance/registerPointOfInterest");
    ROS_WARN_COND(twist_controller_params_.collision_check_frames.size() <= 0,
                  "There are no collision check frames for this manipulator. So nothing will be registered. Ensure parameters are set correctly.");

    for(std::vector<std::string>::const_iterator it = twist_controller_params_.collision_check_frames.begin();
            it != twist_controller_params_.collision_check_frames.end();
            it++)
    {
        ROS_INFO_STREAM("Trying to register for " << *it);
        cob_srvs::SetString r;
        r.request.data = *it;
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
    this->checkSolverAndConstraints(config);
    twist_controller_params_.hardware_interface_type = static_cast<HardwareInterfaceTypes>(config.hardware_interface_type);
    
    twist_controller_params_.numerical_filtering = config.numerical_filtering;
    twist_controller_params_.damping_method = static_cast<DampingMethodTypes>(config.damping_method);
    twist_controller_params_.damping_factor = config.damping_factor;
    twist_controller_params_.lambda_max = config.lambda_max;
    twist_controller_params_.w_threshold = config.w_threshold;
    twist_controller_params_.beta = config.beta;
    twist_controller_params_.eps_damping = config.eps_damping;
    
    twist_controller_params_.solver = static_cast<SolverTypes>(config.solver);
    twist_controller_params_.priority_main = config.priority;

    twist_controller_params_.constraint_jla = static_cast<ConstraintTypesJLA>(config.constraint_jla);
    twist_controller_params_.priority_jla = config.priority_jla;
    twist_controller_params_.k_H_jla = config.k_H_jla;
    double activation_in_percent = config.activation_threshold_jla; // in [%]
    twist_controller_params_.activation_threshold_jla = activation_in_percent / 100.0;
    twist_controller_params_.damping_jla = config.damping_jla;

    twist_controller_params_.constraint_ca = static_cast<ConstraintTypesCA>(config.constraint_ca);
    twist_controller_params_.priority_ca = config.priority_ca;
    twist_controller_params_.k_H_ca = config.k_H_ca;
    twist_controller_params_.activation_threshold_ca = config.activation_threshold_ca; // in [m]
    twist_controller_params_.damping_ca = config.damping_ca;

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

    if(CA_OFF != twist_controller_params_.constraint_ca)
    {
        this->reinitServiceRegistration();
    }
}


void CobTwistController::checkSolverAndConstraints(cob_twist_controller::TwistControllerConfig& config)
{
    bool warning = false;
    SolverTypes solver = static_cast<SolverTypes>(config.solver);
    ConstraintTypesJLA ct_jla = static_cast<ConstraintTypesJLA>(config.constraint_jla);
    ConstraintTypesCA ct_ca = static_cast<ConstraintTypesCA>(config.constraint_ca);
    bool base_compensation = config.base_compensation;
    KinematicExtensionTypes ket = static_cast<KinematicExtensionTypes>(config.kinematic_extension);

    if(base_compensation && BASE_ACTIVE == ket)
    {
        ROS_ERROR("Base cannot be active and compensated at the same time! Setting base compensation back to false ...");
        config.base_compensation = twist_controller_params_.base_compensation = false;
        warning = true;
    }

    if(DEFAULT_SOLVER == solver && (JLA_OFF != ct_jla || CA_OFF != ct_ca))
    {
        ROS_ERROR("The selection of Default solver and a constraint doesn\'t make any sense. Switch settings back ...");
        twist_controller_params_.constraint_jla = JLA_OFF;
        twist_controller_params_.constraint_ca = CA_OFF;
        config.constraint_jla = static_cast<int>(twist_controller_params_.constraint_jla);
        config.constraint_ca = static_cast<int>(twist_controller_params_.constraint_ca);
        warning = true;
    }

    if(WLN == solver && CA_OFF != ct_ca)
    {
        ROS_ERROR("The WLN solution doesn\'t support collision avoidance. Currently WLN is only implemented for Identity and JLA ...");
        twist_controller_params_.constraint_ca = CA_OFF;
        config.constraint_ca = static_cast<int>(twist_controller_params_.constraint_ca);
        warning = true;
    }

    if(GPM == solver && CA_OFF == ct_ca && JLA_OFF == ct_jla)
    {
        ROS_ERROR("You have chosen GPM but without constraints! The behaviour without constraints will be the same like for DEFAULT_SOLVER.");
        warning = true;
    }

    if(TASK_2ND_PRIO == solver && (JLA_ON == ct_jla || CA_OFF == ct_ca))
    {
        ROS_ERROR("The projection of a task into the null space of the main EE task is currently only for the CA constraint supported!");
        twist_controller_params_.constraint_jla = JLA_OFF;
        twist_controller_params_.constraint_ca = CA_ON;
        config.constraint_jla = static_cast<int>(twist_controller_params_.constraint_jla);
        config.constraint_ca = static_cast<int>(twist_controller_params_.constraint_ca);
        warning = true;
    }

    if(!warning)
    {
        ROS_INFO("Parameters seem to be ok.");
    }
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
            if(strcmp(msg->name[i].c_str(), twist_controller_params_.joints[j].c_str()) == 0)
            {
                q_temp(j) = msg->position[i];
                q_dot_temp(j) = msg->velocity[i];
                count++;
                break;
            }
        }
    }

    if(count == twist_controller_params_.joints.size())
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
