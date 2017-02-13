#include <ros/ros.h>

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

  vel_smoother_.reset(new VelocitySmoother(name));
  if (!vel_smoother_->init(nh_p))
  {
    ROS_ERROR_STREAM("Velocity Smoother: initialisation failed [" << name << "]");
    return -1;
  }

  ROS_DEBUG_STREAM("Velocity Smoother: initialised [" << name << "]");
  ros::spin();

  return 0;
}