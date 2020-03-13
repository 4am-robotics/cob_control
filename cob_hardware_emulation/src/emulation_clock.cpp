#include <ctime>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>


class EmulationClock
{
public:
    EmulationClock(boost::asio::io_service& io, double dt, double time_factor):
        dt_(dt),
        time_factor_(time_factor),
        timer_(boost::asio::deadline_timer(io, boost::posix_time::seconds(1/dt)))
    {
        t_ = (double)std::time(NULL);
        pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
        //timer_ = nh_.createTimer(ros::Duration(dt_), &EmulationClock::timer_cb, this);
        timer_.async_wait(boost::bind(&EmulationClock::timer_cb, this));
    }

    //void timer_cb(const ros::TimerEvent& event)
    void timer_cb(const boost::system::error_code& /*e*/)
    {
        t_ += dt_*time_factor_;
        rosgraph_msgs::Clock msg;
        msg.clock = ros::Time(t_);
        pub_.publish(msg);

        timer_.expires_at(timer_.expires_at() + boost::posix_time::seconds(1/dt_));
        timer_.async_wait(boost::bind(&EmulationClock::timer_cb, this));
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    //ros::Timer timer_;
    boost::asio::deadline_timer timer_;
    double t_;
    double dt_;
    double time_factor_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "emulation_clock");
    ros::NodeHandle nh;
    double dt;
    double time_factor;
    nh.param<double>("/emulation_dt", dt, 0.01);
    nh.param<double>("/emulation_time_factor", time_factor, 1.0);

    boost::asio::io_service io;
    EmulationClock ec(io, dt, time_factor);
    io.run();
}
