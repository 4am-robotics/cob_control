#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

class ForwardCommandExecutionTester
{
public:
    ForwardCommandExecutionTester()
    {
        dof_ = 2;
        idx_ = 1;

        output_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/torso/joint_group_position_controller/command", 1);

        ros::Duration(1.0).sleep();
    }


    ~ForwardCommandExecutionTester()
    {}

    void run()
    {
        ros::Rate r(100.0);

        ros::Time time = ros::Time::now();
        ros::Time start_time = time;
        double x = 0.0;

        double a = 0.6, b = 0.4, c = 0, d = 0;      // torso_2dof
        
        std_msgs::Float64MultiArray command_msg;
        command_msg.data.assign(dof_, 0.0);

        while (ros::ok())
        {
            time = ros::Time::now();
            x = (time - start_time).toSec();
            
            double vel = a*sin(b*x+c) + d;
            
            command_msg.data[idx_] = vel;
            
            output_pub_.publish(command_msg);
            
            ros::spinOnce();
            r.sleep();
        }
    }


    ros::NodeHandle nh_;
    ros::Publisher output_pub_;
    unsigned int dof_;
    unsigned int idx_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_forward_command_execution_node");

    ForwardCommandExecutionTester fcet;
    fcet.run();
    ros::spin();
    return 0;
}
