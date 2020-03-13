#include <ctime>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>



class EmulationClock
{
public:
    EmulationClock() {
        t_ = (double)std::time(NULL);
        nh_.param<double>("/emulation_dt", dt_, 0.01);
        nh_.param<double>("/emulation_time_factor", time_factor_, 1.0);
        pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
        timer_ = nh_.createTimer(ros::Duration(dt_), &EmulationClock::timer_cb, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Timer timer_;
    double t_;
    double dt_;
    double time_factor_;

    void timer_cb(const ros::TimerEvent& event)
    {
        t_ += dt_*time_factor_;
        rosgraph_msgs::Clock msg;
        msg.clock = ros::Time(t_);
        pub_.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "emulation_clock");
    EmulationClock ec;

    ros::spin();
}
