/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef COB_FRAME_TRACKER_H
#define COB_FRAME_TRACKER_H

#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <cob_srvs/SetString.h>

#include <actionlib/server/simple_action_server.h>
#include <cob_frame_tracker/FrameTrackerConfig.h>
#include <cob_frame_tracker/FrameTrackingAction.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include <control_toolbox/pid.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <boost/thread/mutex.hpp>


typedef actionlib::SimpleActionServer<cob_frame_tracker::FrameTrackingAction> SAS_FrameTrackingAction_t;

struct HoldTf
{
    tf::StampedTransform transform_tf;
    bool hold;
};


class CobFrameTracker
{
public:
    CobFrameTracker()
    {
        ht_.hold = false;
    }

    ~CobFrameTracker()
    {
        jntToCartSolver_vel_.reset();
        as_.reset();
        reconfigure_server_.reset();
    }

    bool initialize();
    void run(const ros::TimerEvent& event);

    void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    bool startTrackingCallback(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response);
    bool startLookatCallback(cob_srvs::SetString::Request& request, cob_srvs::SetString::Response& response);
    bool stopCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

    bool getTransform(const std::string& from, const std::string& to, tf::StampedTransform& stamped_tf);

    void publishZeroTwist();
    void publishTwist(ros::Duration period, bool do_publish = true);
    void publishHoldTwist(const ros::Duration& period);

    /// Action interface
    void goalCB();
    void preemptCB();
    void action_success();
    void action_abort();

private:
    HoldTf ht_;

    double update_rate_;
    ros::Timer timer_;

    bool tracking_;
    bool tracking_goal_;
    bool lookat_;
    std::string chain_base_link_;
    std::string chain_tip_link_;
    std::string lookat_focus_frame_;
    std::string tracking_frame_;    // the frame tracking the target (i.e. chain_tip or lookat_focus)
    std::string target_frame_;      // the frame to be tracked

    double max_vel_lin_;
    double max_vel_rot_;

    std::vector<std::string> joints_;
    unsigned int dof_;

    bool movable_trans_;
    bool movable_rot_;

    control_toolbox::Pid pid_controller_trans_x_;       /** < Internal PID controller. */
    control_toolbox::Pid pid_controller_trans_y_;
    control_toolbox::Pid pid_controller_trans_z_;

    control_toolbox::Pid pid_controller_rot_x_;         /** < Internal PID controller. */
    control_toolbox::Pid pid_controller_rot_y_;
    control_toolbox::Pid pid_controller_rot_z_;

    /// KDL Conversion
    KDL::Chain chain_;
    KDL::JntArray last_q_;
    KDL::JntArray last_q_dot_;
    boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jntToCartSolver_vel_;

    tf::TransformListener tf_listener_;

    ros::Subscriber jointstate_sub_;
    ros::Publisher twist_pub_;
    ros::Publisher error_pub_;

    ros::ServiceServer start_tracking_server_;
    ros::ServiceServer start_lookat_server_;
    ros::ServiceServer stop_server_;
    ros::ServiceClient reconfigure_client_;

    /// Action interface
    std::string action_name_;
    boost::shared_ptr<SAS_FrameTrackingAction_t> as_;

    cob_frame_tracker::FrameTrackingFeedback action_feedback_;
    cob_frame_tracker::FrameTrackingResult action_result_;

    boost::recursive_mutex reconfig_mutex_;
    boost::shared_ptr< dynamic_reconfigure::Server<cob_frame_tracker::FrameTrackerConfig> > reconfigure_server_;
    void reconfigureCallback(cob_frame_tracker::FrameTrackerConfig& config, uint32_t level);

    /// ABORTION CRITERIA:
    int checkStatus();
    bool checkInfinitesimalTwist(const KDL::Twist current);
    bool checkCartDistanceViolation(const double dist, const double rot);
    bool checkTwistViolation(const KDL::Twist current, const KDL::Twist target);

    int checkServiceCallStatus();

    bool stop_on_goal_;
    double tracking_duration_;
    ros::Time tracking_start_time_;

    bool enable_abortion_checking_;
    double cart_min_dist_threshold_lin_;
    double cart_min_dist_threshold_rot_;
    double twist_dead_threshold_lin_;
    double twist_dead_threshold_rot_;
    double twist_deviation_threshold_lin_;
    double twist_deviation_threshold_rot_;

    KDL::Twist current_twist_;
    KDL::Twist target_twist_;

    double cart_distance_;
    double rot_distance_;

    unsigned int abortion_counter_;
    unsigned int max_abortions_;
};

#endif
