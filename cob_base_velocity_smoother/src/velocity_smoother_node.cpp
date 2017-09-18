/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


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
