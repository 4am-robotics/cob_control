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
 *   ROS stack name: cob_driver
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
#ifndef COB_TWIST_CONTROLLER_H
#define COB_TWIST_CONTROLLER_H

#include <ros/ros.h>

#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <dynamic_reconfigure/server.h>

#include <cob_twist_controller/TwistControllerConfig.h>
#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include <cob_twist_controller/inverse_differential_kinematics_solver.h>
#include "cob_twist_controller/limiters/limiter.h"
#include "cob_twist_controller/interface_types/interface_type.h"
#include "cob_twist_controller/callback_data_mediator.h"

class CobTwistController
{
private:
    ros::NodeHandle nh_;
    
    ros::Subscriber jointstate_sub;
    
    ros::Subscriber twist_sub;
    ros::Subscriber twist_stamped_sub;

    ros::Subscriber odometry_sub;

    ros::Subscriber obstacle_distance_sub_;

    KDL::Chain chain_;
    KDL::JntArray last_q_;
    KDL::JntArray last_q_dot_;
    KDL::Twist twist_odometry_cb_;

    std::string chain_base_link_;
    std::string chain_tip_link_;
    std::vector<std::string> joints_;

    TwistControllerParams twist_controller_params_;
    void initInvDiffKinSolverParams();

    boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jntToCartSolver_vel_;
    boost::shared_ptr<InverseDifferentialKinematicsSolver> p_inv_diff_kin_solver_;
    boost::shared_ptr<LimiterContainer> limiters_;
    boost::shared_ptr<InterfaceBase> interface_;
    
    CallbackDataMediator callback_data_mediator_;

    tf::TransformListener tf_listener_;
    std::vector<double> initial_pos_;


public:
    CobTwistController()
    {
        this->twist_controller_params_.keep_direction = true;
        this->twist_controller_params_.enforce_pos_limits = true;
        this->twist_controller_params_.enforce_vel_limits = true;
        this->twist_controller_params_.base_active = false;
        this->twist_controller_params_.base_compensation = false;
    }

    ~CobTwistController()
    {
        this->p_inv_diff_kin_solver_.reset();
        this->limiters_.reset();
    }

    bool initialize();
    void run();

    void reinitServiceRegistration();

    void reconfigureCallback(cob_twist_controller::TwistControllerConfig &config, uint32_t level);
    void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void solveTwist(KDL::Twist twist);

    boost::recursive_mutex reconfig_mutex_;
    boost::shared_ptr< dynamic_reconfigure::Server<cob_twist_controller::TwistControllerConfig> > reconfigure_server_;

};
#endif
