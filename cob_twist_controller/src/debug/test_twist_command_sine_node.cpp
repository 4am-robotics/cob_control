#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class TwistCommandExecutionTester
{
public:
    TwistCommandExecutionTester()
    {
        output_pub_ = nh_.advertise<geometry_msgs::Twist>("command", 1);

        ros::Duration(1.0).sleep();
    }


    ~TwistCommandExecutionTester()
    {}

    void run()
    {
        ros::Rate r(50.0);

        ros::Time time = ros::Time::now();
        ros::Time start_time = time;
        double x = 0.0;

        double a = 0.6, b = 0.4, c = 0, d = 0;
        
        geometry_msgs::Twist command_msg;

        while (ros::ok())
        {
            time = ros::Time::now();
            x = (time - start_time).toSec();
            
            double vel = a*sin(b*x+c) + d;
            
            command_msg.angular.z = vel;
            
            output_pub_.publish(command_msg);
            
            ros::spinOnce();
            r.sleep();
        }
    }


    ros::NodeHandle nh_;
    ros::Publisher output_pub_;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_twist_command_execution_node");

    TwistCommandExecutionTester tcet;
    tcet.run();
    ros::spin();
    return 0;
}
