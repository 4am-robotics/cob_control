// ROS includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <cob_twist_action/cob_twist_actionConfig.h>

// ROS message includes
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <cob_twist_action/TwistAction.h>

// other includes
#include <cob_twist_action_common.cpp>


class cob_twist_action_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<cob_twist_action::cob_twist_actionConfig> server;
    dynamic_reconfigure::Server<cob_twist_action::cob_twist_actionConfig>::CallbackType f;

    ros::Publisher twist_action_;
    ros::Subscriber twist_command_;
    ros::Subscriber joint_state_reader_;
    actionlib::SimpleActionServer<cob_twist_action::TwistAction> as_perform_twist_;

    cob_twist_action_data component_data_;
    cob_twist_action_config component_config_;
    cob_twist_action_impl component_implementation_;

    cob_twist_action_ros() : np_("~")
    , as_perform_twist_(n_, "perform_twist", boost::bind(&cob_twist_action_impl::callback_perform_twist_, &component_implementation_, _1, &as_perform_twist_), false)
    {
        f = boost::bind(&cob_twist_action_ros::configure_callback, this, _1, _2);
        server.setCallback(f);
        as_perform_twist_.start();

        twist_action_ = n_.advertise<geometry_msgs::Twist>("twist_action", 1);
        twist_command_ = n_.subscribe("twist_command", 1, &cob_twist_action_ros::topicCallback_twist_command, this);
        joint_state_reader_ = n_.subscribe("joint_state_reader", 1, &cob_twist_action_ros::topicCallback_joint_state_reader, this);

        np_.param("x_dot_max_deviation_lin", component_config_.x_dot_max_deviation_lin, (double)1.0);
        np_.param("target_tracking_frame", component_config_.target_tracking_frame, (std::string)"");
        np_.param("x_dot_max_deviation_rot", component_config_.x_dot_max_deviation_rot, (double)3.15);
    }
    void topicCallback_twist_command(const geometry_msgs::Twist::ConstPtr& msg)
    {
        component_data_.in_twist_command = *msg;
        update(); //call only if defined as event port, not called if it is an data port
    }
    void topicCallback_joint_state_reader(const sensor_msgs::JointState::ConstPtr& msg)
    {
        component_data_.in_joint_state_reader = *msg;
    }

    void configure_callback(cob_twist_action::cob_twist_actionConfig &config, uint32_t level)
    {
        component_config_.x_dot_max_deviation_lin = config.x_dot_max_deviation_lin;
        component_config_.target_tracking_frame = config.target_tracking_frame;
        component_config_.x_dot_max_deviation_rot = config.x_dot_max_deviation_rot;
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_twist_action_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_twist_action_active)
            twist_action_.publish(component_data_.out_twist_action);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "cob_twist_action");

    cob_twist_action_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
