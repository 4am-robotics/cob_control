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


#ifndef COB_OMNI_DRIVE_CONTROLLER_ROS_H_
#define COB_OMNI_DRIVE_CONTROLLER_ROS_H_

#include <ros/ros.h>
#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>

namespace cob_omni_drive_controller{

bool parseWheelParams(std::vector<UndercarriageGeom::WheelParams> &params, const ros::NodeHandle &nh, bool read_urdf = true);
bool parseWheelParams(std::vector<UndercarriageDirectCtrl::WheelParams> &params, const ros::NodeHandle &nh, bool read_urdf = true);
bool parseWheelParams(std::vector<UndercarriageCtrl::WheelParams> &params, const ros::NodeHandle &nh, bool read_urdf = true);

}
#endif
