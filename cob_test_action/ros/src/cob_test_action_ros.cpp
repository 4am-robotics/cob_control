// ROS includes
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <cob_test_action/cob_test_actionConfig.h>

// ROS message includes
#include <std_msgs/String.h>
#include <cob_test_action/TriggerAction.h>

// other includes
#include <cob_test_action_common.cpp>


class cob_test_action_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<cob_test_action::cob_test_actionConfig> server;
    dynamic_reconfigure::Server<cob_test_action::cob_test_actionConfig>::CallbackType f;

    ros::Subscriber sub_;
    actionlib::SimpleActionServer<cob_test_action::TriggerAction> as_TriggerTracker_;

    cob_test_action_data component_data_;
    cob_test_action_config component_config_;
    cob_test_action_impl component_implementation_;

    cob_test_action_ros() : np_("~")
    , as_TriggerTracker_(n_, "TriggerTracker", boost::bind(&cob_test_action_impl::callback_TriggerTracker_, &component_implementation_, _1, &as_TriggerTracker_), false)
    {
        f = boost::bind(&cob_test_action_ros::configure_callback, this, _1, _2);
        server.setCallback(f);
        as_TriggerTracker_.start();


        sub_ = n_.subscribe("sub", 1, &cob_test_action_ros::topicCallback_sub, this);

        np_.param("deviation", component_config_.deviation, (int)10);
    }
    void topicCallback_sub(const std_msgs::String::ConstPtr& msg)
    {
        component_data_.in_sub = *msg;
        update(); //call only if defined as event port, not called if it is an data port
    }

    void configure_callback(cob_test_action::cob_test_actionConfig &config, uint32_t level)
    {
        component_config_.deviation = config.deviation;
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "cob_test_action");

    cob_test_action_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
