#include <ctime>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>


class EmulationClock
{
public:
    EmulationClock(boost::asio::io_service& io, int dt_ms, double time_factor)
        : dt_ms_(dt_ms),
          time_factor_(time_factor),
          timer_(io, boost::posix_time::milliseconds(dt_ms))
    {
        t_ = (double)std::time(NULL);
        pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
        timer_.async_wait(boost::bind(&EmulationClock::timer_cb, this));
    }

    void timer_cb()
    {
        t_ += time_factor_*dt_ms_/1000;
        ROS_DEBUG_STREAM("WALL: "<<(double)std::time(NULL)<<"; ROS: "<<t_<<"; dt_ms: "<<dt_ms_<<"; time_factor: "<<time_factor_);
        rosgraph_msgs::Clock msg;
        msg.clock = ros::Time(t_);
        pub_.publish(msg);

        timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(dt_ms_));
        timer_.async_wait(boost::bind(&EmulationClock::timer_cb, this));
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;

    int dt_ms_;
    double time_factor_;
    double t_;

private:
    boost::asio::deadline_timer timer_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "emulation_clock");
    ros::NodeHandle nh;

    int dt_ms;
    double time_factor;
    nh.param<int>("/emulation_dt_ms", dt_ms, 10);
    nh.param<double>("/emulation_time_factor", time_factor, 1.0);

    boost::asio::io_service io;
    EmulationClock ec(io, dt_ms, time_factor);
    io.run();

    return 0;
}
