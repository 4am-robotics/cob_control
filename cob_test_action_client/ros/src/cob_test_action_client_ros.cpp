// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cob_test_action_client/cob_test_action_clientConfig.h>

// ROS message includes
#include <std_msgs/String.h>

// other includes
#include <cob_test_action_client_common.cpp>


class cob_test_action_client_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<cob_test_action_client::cob_test_action_clientConfig> server;
    dynamic_reconfigure::Server<cob_test_action_client::cob_test_action_clientConfig>::CallbackType f;

    ros::Subscriber saub_;

    cob_test_action_client_data component_data_;
    cob_test_action_client_config component_config_;
    cob_test_action_client_impl component_implementation_;

    cob_test_action_client_ros() : np_("~")
    {
        f = boost::bind(&cob_test_action_client_ros::configure_callback, this, _1, _2);
        server.setCallback(f);


        saub_ = n_.subscribe("saub", 1, &cob_test_action_client_ros::topicCallback_saub, this);

    }
    void topicCallback_saub(const std_msgs::String::ConstPtr& msg)
    {
        component_data_.in_saub = *msg;
    }

    void configure_callback(cob_test_action_client::cob_test_action_clientConfig &config, uint32_t level)
    {
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

    ros::init(argc, argv, "cob_test_action_client");

    cob_test_action_client_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
