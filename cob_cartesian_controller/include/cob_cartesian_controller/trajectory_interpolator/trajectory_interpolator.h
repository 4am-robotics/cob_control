/*
 * trajectory_interpolator.h
 *
 *  Created on: Jun 30, 2015
 *      Author: fxm-cm
 */

#ifndef COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_TRAJECTORY_INTERPOLATOR_TRAJECTORY_INTERPOLATOR_H_
#define COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_TRAJECTORY_INTERPOLATOR_TRAJECTORY_INTERPOLATOR_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator.h>

class TrajectoryInterpolator
{
public:
    TrajectoryInterpolator(double update_rate)
    :   TPG_(TrajectoryProfileGenerator(update_rate))
    {}

    ~TrajectoryInterpolator(){}

    void linear_interpolation(  std::vector <geometry_msgs::Pose>& poseVector,
                                geometry_msgs::Pose start, geometry_msgs::Pose end,
                                double VelMax, double AcclMax, std::string profile,bool justRotate);

    void circular_interpolation(std::vector<geometry_msgs::Pose>& poseVector,
                                double M_x,double M_y,double M_z,
                                double M_roll,double M_pitch,double M_yaw,
                                double startAngle, double endAngle,double r, double VelMax, double AcclMax,
                                std::string profile);
private:
    TrajectoryProfileGenerator TPG_;

};



#endif /* COB_CONTROL_COB_CARTESIAN_CONTROLLER_INCLUDE_COB_CARTESIAN_CONTROLLER_TRAJECTORY_INTERPOLATOR_TRAJECTORY_INTERPOLATOR_H_ */
