
#ifndef COB_OMNI_DRIVE_CONTROLLER_ROS_H_
#define COB_OMNI_DRIVE_CONTROLLER_ROS_H_

#include <ros/ros.h>
#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>

namespace cob_omni_drive_controller{
    
bool parseWheelParams(std::vector<UndercarriageGeom::WheelParams> &params, const ros::NodeHandle &nh, bool read_urdf = true);
bool parseWheelParams(std::vector<UndercarriageCtrl::WheelParams> &params, const ros::NodeHandle &nh, bool read_urdf = true);

}
#endif
