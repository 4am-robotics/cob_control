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
#include <string>
#include <vector>
#include <limits>
#include <ros/ros.h>

#include <cob_twist_controller/cob_twist_controller.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cob_srvs/SetString.h>

#include <Eigen/Dense>

bool CobTwistController::initialize()
{
    ros::NodeHandle nh_twist("twist_controller");

    // JointNames
    if (!nh_.getParam("joint_names", twist_controller_params_.joints))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }

    twist_controller_params_.dof = twist_controller_params_.joints.size();

    // Chain
    if (!nh_.getParam("chain_base_link", twist_controller_params_.chain_base_link))
    {
        ROS_ERROR("Parameter 'chain_base_link' not set");
        return false;
    }

    if (!nh_.getParam("chain_tip_link", twist_controller_params_.chain_tip_link))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    // links of the chain to be considered for collision avoidance
    if (!nh_twist.getParam("collision_check_links", twist_controller_params_.collision_check_links))
    {
        ROS_WARN_STREAM("Parameter 'collision_check_links' not set. Collision Avoidance constraint will not do anything.");
        twist_controller_params_.collision_check_links.clear();
    }

    /// parse robot_description and generate KDL chains
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromParam("/robot_description", my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    my_tree.getChain(twist_controller_params_.chain_base_link, twist_controller_params_.chain_tip_link, chain_);
    if (chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    /// parse robot_description and set velocity limits
    urdf::Model model;
    if (!model.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file for JointLimits");
        return false;
    }

    for (uint16_t i = 0; i < twist_controller_params_.dof; i++)
    {
        twist_controller_params_.limiter_params.limits_min.push_back(model.getJoint(twist_controller_params_.joints[i])->limits->lower);
        twist_controller_params_.limiter_params.limits_max.push_back(model.getJoint(twist_controller_params_.joints[i])->limits->upper);
        twist_controller_params_.limiter_params.limits_vel.push_back(model.getJoint(twist_controller_params_.joints[i])->limits->velocity);
    }

    // Currently not supported yet
    if ((!nh_twist.getParam("limits_acc", twist_controller_params_.limiter_params.limits_acc)) || (twist_controller_params_.limiter_params.limits_acc.size() != twist_controller_params_.dof))
    {
        // ROS_ERROR("Parameter 'limits_acc' not set or dimensions do not match! Not limiting acceleration!");
        for (uint16_t i = 0; i < twist_controller_params_.dof; i++)
        {
            twist_controller_params_.limiter_params.limits_acc.push_back(std::numeric_limits<double>::max());
        }
    }

    // Configure Lookat Extenstion (Offset and Axis) --- not dynamic-reconfigurable
    if (nh_twist.hasParam("lookat_axis_type"))
    {
        int lookat_axis_type;
        nh_twist.getParam("lookat_axis_type", lookat_axis_type);
        twist_controller_params_.lookat_offset.lookat_axis_type = static_cast<LookatAxisTypes>(lookat_axis_type);
    }
    if (nh_twist.hasParam("lookat_offset"))
    {
        if (nh_twist.hasParam("lookat_offset/translation"))
        {
            twist_controller_params_.lookat_offset.translation_x = nh_twist.param("lookat_offset/translation/x", 0.0);
            twist_controller_params_.lookat_offset.translation_y = nh_twist.param("lookat_offset/translation/y", 0.0);
            twist_controller_params_.lookat_offset.translation_z = nh_twist.param("lookat_offset/translation/z", 0.0);
        }
        if (nh_twist.hasParam("lookat_offset/rotation"))
        {
            twist_controller_params_.lookat_offset.rotation_x = nh_twist.param("lookat_offset/rotation/x", 0.0);
            twist_controller_params_.lookat_offset.rotation_y = nh_twist.param("lookat_offset/rotation/y", 0.0);
            twist_controller_params_.lookat_offset.rotation_z = nh_twist.param("lookat_offset/rotation/z", 0.0);
            twist_controller_params_.lookat_offset.rotation_w = nh_twist.param("lookat_offset/rotation/w", 1.0);
        }
    }

    twist_controller_params_.frame_names.clear();
    for (uint16_t i = 0; i < chain_.getNrOfSegments(); ++i)
    {
        twist_controller_params_.frame_names.push_back(chain_.getSegment(i).getName());
    }
    register_link_client_ = nh_.serviceClient<cob_srvs::SetString>("obstacle_distance/registerLinkOfInterest");
    register_link_client_.waitForExistence(ros::Duration(5.0));
    twist_controller_params_.constraint_ca = CA_OFF;

    /// initialize configuration control solver
    p_inv_diff_kin_solver_.reset(new InverseDifferentialKinematicsSolver(twist_controller_params_, chain_, callback_data_mediator_));
    p_inv_diff_kin_solver_->resetAll(twist_controller_params_);

    /// Setting up dynamic_reconfigure server for the TwistControlerConfig parameters
    reconfigure_server_.reset(new dynamic_reconfigure::Server<cob_twist_controller::TwistControllerConfig>(reconfig_mutex_, nh_twist));
    reconfigure_server_->setCallback(boost::bind(&CobTwistController::reconfigureCallback,   this, _1, _2));

    /// initialize variables and current joint values and velocities
    this->joint_states_.current_q_ = KDL::JntArray(chain_.getNrOfJoints());
    this->joint_states_.current_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());
    this->joint_states_.last_q_ = KDL::JntArray(chain_.getNrOfJoints());
    this->joint_states_.last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());

    /// give tf_listener some time to fill tf-cache
    ros::Duration(1.0).sleep();

    /// initialize ROS interfaces
    obstacle_distance_sub_ = nh_.subscribe("obstacle_distance", 1, &CallbackDataMediator::distancesToObstaclesCallback, &callback_data_mediator_);
    jointstate_sub_ = nh_.subscribe("joint_states", 1, &CobTwistController::jointstateCallback, this);
    twist_sub_ = nh_twist.subscribe("command_twist", 1, &CobTwistController::twistCallback, this);
    twist_stamped_sub_ = nh_twist.subscribe("command_twist_stamped", 1, &CobTwistController::twistStampedCallback, this);

    odometry_sub_ = nh_.subscribe("base/odometry", 1, &CobTwistController::odometryCallback, this);

    this->controller_interface_.reset(ControllerInterfaceBuilder::createControllerInterface(this->nh_, this->twist_controller_params_, this->joint_states_));

    /// publisher for visualizing current twist direction
    twist_direction_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("twist_direction", 1);

    ROS_INFO_STREAM(nh_.getNamespace() << "/twist_controller...initialized!");
    return true;
}

bool CobTwistController::registerCollisionLinks()
{
    ROS_WARN_COND(twist_controller_params_.collision_check_links.size() <= 0,
                  "No collision_check_links set for this chain. Nothing will be registered. Ensure parameters are set correctly.");

    for (std::vector<std::string>::const_iterator it = twist_controller_params_.collision_check_links.begin();
         it != twist_controller_params_.collision_check_links.end();
         it++)
    {
        ROS_INFO_STREAM("Trying to register for " << *it);
        cob_srvs::SetString r;
        r.request.data = *it;
        if (register_link_client_.call(r))
        {
            ROS_INFO_STREAM("Called registration service with success: " << int(r.response.success) << ". Got message: " << r.response.message);
            if (!r.response.success)
            {
                return false;
            }
        }
        else
        {
            ROS_WARN_STREAM("Failed to call registration service for namespace: " << nh_.getNamespace());
            return false;
        }
    }

    return true;
}

void CobTwistController::reconfigureCallback(cob_twist_controller::TwistControllerConfig& config, uint32_t level)
{
    this->checkSolverAndConstraints(config);

    twist_controller_params_.controller_interface = static_cast<ControllerInterfaceTypes>(config.controller_interface);
    twist_controller_params_.integrator_smoothing = config.integrator_smoothing;

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
    const double activation_jla_in_percent = config.activation_threshold_jla;
    const double activation_buffer_jla_in_percent = config.activation_buffer_jla;
    const double critical_jla_in_percent = config.critical_threshold_jla;
    twist_controller_params_.thresholds_jla.activation =  activation_jla_in_percent / 100.0;
    twist_controller_params_.thresholds_jla.activation_with_buffer = twist_controller_params_.thresholds_jla.activation * (1.0 + activation_buffer_jla_in_percent / 100.0);
    twist_controller_params_.thresholds_jla.critical =  critical_jla_in_percent / 100.0;
    twist_controller_params_.damping_jla = config.damping_jla;

    twist_controller_params_.constraint_ca = static_cast<ConstraintTypesCA>(config.constraint_ca);
    twist_controller_params_.priority_ca = config.priority_ca;
    twist_controller_params_.k_H_ca = config.k_H_ca;
    const double activaton_buffer_ca_in_percent = config.activation_buffer_ca;
    twist_controller_params_.thresholds_ca.activation = config.activation_threshold_ca;  // in [m]
    twist_controller_params_.thresholds_ca.activation_with_buffer = twist_controller_params_.thresholds_ca.activation * (1.0 + activaton_buffer_ca_in_percent / 100.0);
    twist_controller_params_.thresholds_ca.critical = config.critical_threshold_ca;  // in [m]
    twist_controller_params_.damping_ca = config.damping_ca;

    twist_controller_params_.k_H = config.k_H;

    twist_controller_params_.eps_truncation = config.eps_truncation;

    twist_controller_params_.limiter_params.keep_direction = config.keep_direction;
    twist_controller_params_.limiter_params.enforce_pos_limits = config.enforce_pos_limits;
    twist_controller_params_.limiter_params.enforce_vel_limits = config.enforce_vel_limits;
    twist_controller_params_.limiter_params.enforce_acc_limits = config.enforce_acc_limits;
    twist_controller_params_.limiter_params.limits_tolerance = config.limits_tolerance;

    twist_controller_params_.kinematic_extension = static_cast<KinematicExtensionTypes>(config.kinematic_extension);
    twist_controller_params_.extension_ratio = config.extension_ratio;

    this->controller_interface_.reset(ControllerInterfaceBuilder::createControllerInterface(this->nh_, this->twist_controller_params_, this->joint_states_));

    p_inv_diff_kin_solver_->resetAll(this->twist_controller_params_);
}

void CobTwistController::checkSolverAndConstraints(cob_twist_controller::TwistControllerConfig& config)
{
    bool warning = false;
    SolverTypes solver = static_cast<SolverTypes>(config.solver);

    if (DEFAULT_SOLVER == solver && (JLA_OFF != static_cast<ConstraintTypesJLA>(config.constraint_jla) || CA_OFF != static_cast<ConstraintTypesCA>(config.constraint_ca)))
    {
        ROS_ERROR("The selection of Default solver and a constraint doesn\'t make any sense. Switch settings back ...");
        twist_controller_params_.constraint_jla = JLA_OFF;
        twist_controller_params_.constraint_ca = CA_OFF;
        config.constraint_jla = static_cast<int>(twist_controller_params_.constraint_jla);
        config.constraint_ca = static_cast<int>(twist_controller_params_.constraint_ca);
        warning = true;
    }

    if (WLN == solver && CA_OFF != static_cast<ConstraintTypesCA>(config.constraint_ca))
    {
        ROS_ERROR("The WLN solution doesn\'t support collision avoidance. Currently WLN is only implemented for Identity and JLA ...");
        twist_controller_params_.constraint_ca = CA_OFF;
        config.constraint_ca = static_cast<int>(twist_controller_params_.constraint_ca);
        warning = true;
    }

    if (GPM == solver && CA_OFF == static_cast<ConstraintTypesCA>(config.constraint_ca) && JLA_OFF == static_cast<ConstraintTypesJLA>(config.constraint_jla))
    {
        ROS_ERROR("You have chosen GPM but without constraints! The behaviour without constraints will be the same like for DEFAULT_SOLVER.");
        warning = true;
    }

    if (TASK_2ND_PRIO == solver && (JLA_ON == static_cast<ConstraintTypesJLA>(config.constraint_jla) || CA_OFF == static_cast<ConstraintTypesCA>(config.constraint_ca)))
    {
        ROS_ERROR("The projection of a task into the null space of the main EE task is currently only for the CA constraint supported!");
        twist_controller_params_.constraint_jla = JLA_OFF;
        twist_controller_params_.constraint_ca = CA_ON;
        config.constraint_jla = static_cast<int>(twist_controller_params_.constraint_jla);
        config.constraint_ca = static_cast<int>(twist_controller_params_.constraint_ca);
        warning = true;
    }

    if (CA_OFF != static_cast<ConstraintTypesCA>(config.constraint_ca))
    {
        if (!register_link_client_.exists())
        {
            ROS_ERROR("ServiceServer 'obstacle_distance/registerLinkOfInterest' does not exist. CA not possible");
            twist_controller_params_.constraint_ca = CA_OFF;
            config.constraint_ca = static_cast<int>(twist_controller_params_.constraint_ca);
            warning = true;
        }
        else if (twist_controller_params_.constraint_ca != static_cast<ConstraintTypesCA>(config.constraint_ca))
        {
            ROS_INFO("Collision Avoidance has been activated! Register links!");
            if (!this->registerCollisionLinks())
            {
                ROS_ERROR("Registration of links failed. CA not possible");
                twist_controller_params_.constraint_ca = CA_OFF;
                config.constraint_ca = static_cast<int>(twist_controller_params_.constraint_ca);
                warning = true;
            }
        }
    }

    if (twist_controller_params_.limiter_params.limits_tolerance <= DIV0_SAFE)
    {
        ROS_ERROR("The limits_tolerance for enforce limits is smaller than DIV/0 threshold. Therefore both enforce_limits are set to false!");
        twist_controller_params_.limiter_params.enforce_pos_limits = config.enforce_pos_limits = false;
        twist_controller_params_.limiter_params.enforce_vel_limits = config.enforce_vel_limits = false;
        twist_controller_params_.limiter_params.enforce_acc_limits = config.enforce_acc_limits = false;
    }

    if (!warning)
    {
        ROS_INFO("Parameters seem to be ok.");
    }
}

/// Orientation of twist_stamped_msg is with respect to coordinate system given in header.frame_id
void CobTwistController::twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    tf::StampedTransform transform_tf;
    KDL::Frame frame;
    KDL::Twist twist, twist_transformed;

    try
    {
        tf_listener_.lookupTransform(twist_controller_params_.chain_base_link, msg->header.frame_id, ros::Time(0), transform_tf);
        frame.M = KDL::Rotation::Quaternion(transform_tf.getRotation().x(), transform_tf.getRotation().y(), transform_tf.getRotation().z(), transform_tf.getRotation().w());
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("CobTwistController::twistStampedCallback: \n%s", ex.what());
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
    ros::Time start, end;
    start = ros::Time::now();

    visualizeTwist(twist);

    KDL::JntArray q_dot_ik(chain_.getNrOfJoints());

    if (twist_controller_params_.kinematic_extension == BASE_COMPENSATION)
    {
        twist = twist - twist_odometry_cb_;
    }

    int ret_ik = p_inv_diff_kin_solver_->CartToJnt(this->joint_states_,
                                                   twist,
                                                   q_dot_ik);

    if (0 != ret_ik)
    {
        ROS_ERROR("No Vel-IK found!");
    }
    else
    {
        this->controller_interface_->processResult(q_dot_ik, this->joint_states_.current_q_);
    }

    end = ros::Time::now();
    // ROS_INFO_STREAM("solveTwist took " << (end-start).toSec() << " seconds");
}

void CobTwistController::visualizeTwist(KDL::Twist twist)
{
    std::string tracking_frame = twist_controller_params_.chain_tip_link;
    if (twist_controller_params_.kinematic_extension == LOOKAT)
    {
        tracking_frame = "lookat_focus_frame";
    }

    tf::StampedTransform transform_tf;
    try
    {
        tf_listener_.lookupTransform(twist_controller_params_.chain_base_link, tracking_frame, ros::Time(0), transform_tf);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("CobTwistController::visualizeTwist: \n%s", ex.what());
        return;
    }

    visualization_msgs::Marker marker_vel;
    marker_vel.header.frame_id = twist_controller_params_.chain_base_link;
    marker_vel.header.stamp = ros::Time::now();
    marker_vel.ns = "twist_vel";
    marker_vel.id = 0;
    marker_vel.type = visualization_msgs::Marker::ARROW;
    marker_vel.action = visualization_msgs::Marker::ADD;
    marker_vel.lifetime = ros::Duration(0.1);
    marker_vel.pose.orientation.w = 1.0;

    marker_vel.scale.x = 0.02;
    marker_vel.scale.y = 0.02;
    marker_vel.scale.z = 0.02;

    marker_vel.color.r = 1.0f;
    marker_vel.color.g = 1.0f;
    marker_vel.color.b = 0.0f;
    marker_vel.color.a = 1.0;

    marker_vel.points.resize(2);
    marker_vel.points[0].x = transform_tf.getOrigin().x();
    marker_vel.points[0].y = transform_tf.getOrigin().y();
    marker_vel.points[0].z = transform_tf.getOrigin().z();
    marker_vel.points[1].x = transform_tf.getOrigin().x() + 5.0 * twist.vel.x();
    marker_vel.points[1].y = transform_tf.getOrigin().y() + 5.0 * twist.vel.y();
    marker_vel.points[1].z = transform_tf.getOrigin().z() + 5.0 * twist.vel.z();

    visualization_msgs::Marker marker_rot;
    marker_rot.header.frame_id = twist_controller_params_.chain_base_link;
    marker_rot.header.stamp = ros::Time::now();
    marker_rot.ns = "twist_rot";
    marker_rot.id = 0;
    marker_rot.type = visualization_msgs::Marker::CYLINDER;
    marker_rot.action = visualization_msgs::Marker::ADD;
    marker_rot.lifetime = ros::Duration(0.1);
    marker_rot.pose.position.x = transform_tf.getOrigin().x();
    marker_rot.pose.position.y = transform_tf.getOrigin().y();
    marker_rot.pose.position.z = transform_tf.getOrigin().z();

    tf::Quaternion rot;
    rot.setRPY(twist.rot.x(), twist.rot.y(), twist.rot.z());

    /// calculate rotation between twist-axis and z-axis of cylinder
    tf::Vector3 z_axis = tf::Vector3(0,0,1);
    tf::Vector3 t_axis = rot.getAxis();
    tf::Quaternion temp(0,0,0,1);
    if(z_axis!=t_axis && z_axis!=-t_axis)
    {
        tf::Vector3 cross = z_axis.cross(t_axis);
        temp = tf::Quaternion(cross.x(), cross.y(), cross.z(), (std::sqrt(z_axis.length2() * t_axis.length2()) + z_axis.dot(t_axis)));
        temp = temp.normalized();
    }
    tf::quaternionTFToMsg(temp, marker_rot.pose.orientation);

    marker_rot.scale.x = rot.getAngle();
    marker_rot.scale.y = rot.getAngle();
    marker_rot.scale.z = 0.002;

    marker_rot.color.r = 1.0f;
    marker_rot.color.g = 1.0f;
    marker_rot.color.b = 0.0f;
    marker_rot.color.a = 1.0;

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(marker_vel);
    markers.markers.push_back(marker_rot);

    twist_direction_pub_.publish(markers);
}

void CobTwistController::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = this->joint_states_.current_q_;
    KDL::JntArray q_dot_temp = this->joint_states_.current_q_dot_;
    int count = 0;

    for (uint16_t j = 0; j < twist_controller_params_.dof; j++)
    {
        for (uint16_t i = 0; i < msg->name.size(); i++)
        {
            if (strcmp(msg->name[i].c_str(), twist_controller_params_.joints[j].c_str()) == 0)
            {
                q_temp(j) = msg->position[i];
                q_dot_temp(j) = msg->velocity[i];
                count++;
                break;
            }
        }
    }

    if (count == twist_controller_params_.joints.size())
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

    try
    {
        tf_listener_.waitForTransform(twist_controller_params_.chain_base_link, "base_link", ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform(twist_controller_params_.chain_base_link, "base_link", ros::Time(0), cb_transform_bl);

        tf_listener_.waitForTransform("base_link", twist_controller_params_.chain_tip_link, ros::Time(0), ros::Duration(0.5));
        tf_listener_.lookupTransform("base_link", twist_controller_params_.chain_tip_link, ros::Time(0), bl_transform_ct);

        cb_frame_bl.p = KDL::Vector(cb_transform_bl.getOrigin().x(), cb_transform_bl.getOrigin().y(), cb_transform_bl.getOrigin().z());
        cb_frame_bl.M = KDL::Rotation::Quaternion(cb_transform_bl.getRotation().x(), cb_transform_bl.getRotation().y(), cb_transform_bl.getRotation().z(), cb_transform_bl.getRotation().w());
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("CobTwistController::odometryCallback: \n%s", ex.what());
        return;
    }

    try
    {
        // Calculate tangential twist for angular base movements v = w x r
        Eigen::Vector3d r(bl_transform_ct.getOrigin().x(), bl_transform_ct.getOrigin().y(), bl_transform_ct.getOrigin().z());
        Eigen::Vector3d w(0, 0, msg->twist.twist.angular.z);
        Eigen::Vector3d res = w.cross(r);
        tangential_twist_bl.vel = KDL::Vector(res(0), res(1), res(2));
        tangential_twist_bl.rot = KDL::Vector(0, 0, 0);
    }
    catch(...)
    {
        ROS_ERROR("Error occurred while calculating tangential twist for angular base movements.");
        return;
    }

    tf::twistMsgToKDL(msg->twist.twist, twist_odometry_bl);  // Base Twist

    // transform into chain_base
    twist_odometry_transformed_cb = cb_frame_bl * (twist_odometry_bl + tangential_twist_bl);

    twist_odometry_cb_ = twist_odometry_transformed_cb;
}
