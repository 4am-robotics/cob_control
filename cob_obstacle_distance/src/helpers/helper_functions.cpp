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


#include <string>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include "cob_obstacle_distance/helpers/helper_functions.hpp"


/** Resolves URIs for file:// and package:// paths */
const std::string resolveURI(const std::string& path)
{
    static std::map<std::string, std::string> package_cache;
    std::string uri = path;

    if (uri.find("file://") == 0)
    {
      // Strip off the file://
      uri.erase(0, strlen("file://"));

      // Resolve the mesh path as a file URI
      boost::filesystem::path file_path(uri);
      return file_path.string();
    }
    else if (uri.find("package://") == 0)
    {
      // Strip off the package://
      uri.erase(0, strlen("package://"));

      // Resolve the mesh path as a ROS package URI
      size_t package_end = uri.find("/");
      std::string package = uri.substr(0, package_end);
      std::string package_path;

      // Use the package cache if we have resolved this package before
      std::map<std::string, std::string>::iterator it = package_cache.find(package);
      if (it != package_cache.end())
      {
          package_path = it->second;
      }
      else
      {
          package_path = ros::package::getPath(package);
          package_cache[package] = package_path;
      }

      // Show a warning if the package was not resolved
      if (package_path.empty())
      {
          ROS_ERROR("Unable to find package [%s].", package.c_str());
          return "";
      }

      // Append the remaining relative path
      boost::filesystem::path file_path(package_path);
      uri.erase(0, package_end);
      file_path /= uri;

      // Return the canonical path
      return file_path.string();
    }
    else
    {
      ROS_ERROR("Cannot handle mesh URI type [%s].", uri.c_str());
      return "";
    }
}
