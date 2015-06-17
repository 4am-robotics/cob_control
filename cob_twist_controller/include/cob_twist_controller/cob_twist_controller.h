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

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include <dynamic_reconfigure/server.h>

#include <cob_twist_controller/inverse_differential_kinematics_solver.h>
#include <cob_twist_controller/TwistControllerConfig.h>
#include "cob_twist_controller/cob_twist_controller_data_types.h"
#include "cob_twist_controller/limiters/limiter.h"
#include "cob_twist_controller/callback_data_mediator.h"

#include <deque>

class MovingAverage{
    private:
        std::deque<double> s_;
        std::deque<double> weighting_,temp_;

        int size_;
    public:
            MovingAverage(){
                size_=3;
                calculate_weighting();
            }
            void add_element(double element){
                if(s_.size()<size_){
                    s_.push_front(element);
                }
                else
                {
                    // Drops the first element
                    s_.pop_back();
                    s_.push_front(element);
                }
            }

            double calc_moving_average(){
                double sum = 0;
                for(std::deque<double>::const_iterator i = s_.begin(); i != s_.end(); ++i)
                    sum += *i;

                return sum/s_.size();
            }

            double calc_weighted_moving_average(){
                double sum = 0;
                for(int i = 0; i< s_.size(); i++)
                    sum+=s_[i] * weighting_[i];

                return sum;
            }

            void calculate_weighting(){
                double sum=0;
                double err=0;
                double j=0.0;

                for(int i = 0; i < size_; i++)
                {
                    weighting_.push_back((pow(log(2),j+1) / fakultaet(j+1)));
                    sum += weighting_[i];
                    j += 1.0;
                }
                ROS_WARN("Before: %f",weighting_[0]);
                err = 1 - sum;
                std::deque<double>::iterator i = weighting_.begin();
                *i+=err;
                ROS_WARN("Before: %f",*i);
            }

            double fakultaet(int n)
            {
              if ( n <= 1 )
                return  1;
              else
                return  (double)(n * fakultaet(n-1));
            }

};

class CobTwistController
{
private:

    CallbackDataMediator callback_data_mediator_;


    ros::NodeHandle nh_;
    ros::Time time_;
    ros::Duration period_;
    ros::Subscriber jointstate_sub;
    ros::Subscriber odometry_sub;
    ros::Subscriber twist_sub;

    ros::Subscriber obstacle_distance_sub_;

    ros::Subscriber twist_stamped_sub;
    ros::Subscriber base_sub;
    ros::Publisher vel_pub;
    ros::Publisher pos_pub;
    ros::Publisher base_vel_pub;
    ros::Publisher pub;

    KDL::Chain chain_;
    KDL::Twist twist_odometry_cb_;
    KDL::JntArray last_q_;
    KDL::JntArray last_q_dot_;

    std::string chain_base_link_;
    std::string chain_tip_link_;
    std::vector<std::string> joints_;

    Eigen::VectorXd tracking_err_;

    bool reset_markers_;

    TwistControllerParams twist_controller_params_;

    boost::shared_ptr<InverseDifferentialKinematicsSolver> p_inv_diff_kin_solver_;
    boost::shared_ptr<LimiterContainer> limiters_;
    boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jntToCartSolver_vel_;

    tf::TransformListener tf_listener_;

    ///Debug
    ros::Publisher  debug_base_compensation_visual_tip_pub_,debug_base_compensation_visual_base_pub_,debug_base_compensation_pose_base_pub_,
                    debug_base_compensation_pose_tip_pub_,debug_base_compensation_twist_manipulator_pub_,debug_base_active_twist_manipulator_pub_,
                    debug_base_active_twist_base_pub_,debug_base_active_twist_ee_pub_;
    std::vector<geometry_msgs::Point> point_base_vec_,point_ee_vec_;

    tf::StampedTransform odom_transform_ct,
                         odom_transform_bl,
                         bl_transform_cb,
                         bl_transform_ct,
                         cb_transform_bl;

    KDL::Frame odom_frame_ct,
               odom_frame_bl,
               bl_frame_cb,
               bl_frame_ct,
               cb_frame_bl;

    bool firstIteration_;
    ros::Duration integration_period_;
    ros::Time time_now_;
    ros::Time last_update_time_;

    std::vector<MovingAverage> ma_;
    std::vector<double> old_vel_,old_pos_,initial_pos_,old_vel_2_;
    int iteration_counter_;

    void initInvDiffKinSolverParams();


public:
    CobTwistController():
        reset_markers_(false),firstIteration_(true),iteration_counter_(0)
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
    void baseTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void twistStampedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void solveTwist(KDL::Twist twist);

    ///Debug
    void showMarker(int marker_id,double red, double green, double blue, std::string ns, ros::Publisher pub, std::vector<geometry_msgs::Point> &pos_v);
    void debug();

    boost::recursive_mutex reconfig_mutex_;
    boost::shared_ptr< dynamic_reconfigure::Server<cob_twist_controller::TwistControllerConfig> > reconfigure_server_;

};
#endif
