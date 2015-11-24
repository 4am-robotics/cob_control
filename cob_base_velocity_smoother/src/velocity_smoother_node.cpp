#include <ros/ros.h>
#include <boost/thread.hpp>

#include <cob_base_velocity_smoother/velocity_smoother.h>

using namespace cob_base_velocity_smoother;

std::string unresolvedName(const std::string &name){
    size_t pos = name.find_last_of('/');
    return name.substr(pos + 1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_smoother");
  ros::NodeHandle nh_p("~");
  std::string resolved_name = nh_p.getUnresolvedNamespace(); // this always returns like /robosem/goo_arm - why not unresolved?
  std::string name = unresolvedName(resolved_name); // unresolve it ourselves

  boost::shared_ptr<VelocitySmoother> vel_smoother_;
  boost::shared_ptr<boost::thread>   worker_thread_;

  vel_smoother_.reset(new VelocitySmoother(name));
  if (vel_smoother_->init(nh_p))
  {
    ROS_DEBUG_STREAM("Velocity Smoother: initialised [" << name << "]");
    worker_thread_.reset(new boost::thread(&VelocitySmoother::spin, vel_smoother_));
  }
  else
  {
    ROS_ERROR_STREAM("Velocity Smoother: initialisation failed [" << name << "]");
  }

  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  vel_smoother_->shutdown();
  worker_thread_->join();
  
  return 0;
}