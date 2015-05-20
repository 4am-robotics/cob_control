/****************************************************************
 *
 * Copyright (c) 2014
 *
 * Fraunhofer Institute for Manufacturing Engineering   
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 *                              
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <nav_msgs/Odometry.h>

#ifndef COB_ODOMETRY_TRACKER_H
#define COB_ODOMETRY_TRACKER_H

class OdometryTracker{
    nav_msgs::Odometry odom_;
    double theta_rob_rad_;
public:
    OdometryTracker(const std::string &from = "/wheelodom", const std::string &to = "/base_footprint" , double cov_pose = 0.1, double cov_twist = 0.1) {
        odom_.header.frame_id = from;
        odom_.child_frame_id = to;
        for(int i = 0; i < 6; i++){
            odom_.pose.covariance[i*6+i] = cov_pose;
            odom_.twist.covariance[6*i+i] = cov_twist;
        }
        // odom_.pose.pose.position.z = 0.0;
        // odom_.twist.twist.linear.z = 0.0;
        // odom_.twist.twist.angular.x = 0.0;
        // odom_.twist.twist.angular.y = 0.0;
        init(ros::Time::now());
    }
    void init(const ros::Time &now){
        theta_rob_rad_ = 0;

        odom_.header.stamp = now;

        odom_.twist.twist.linear.x = 0;
        odom_.twist.twist.linear.y = 0;
        odom_.twist.twist.angular.z = 0;

        odom_.pose.pose.position.x = 0;
        odom_.pose.pose.position.y = 0;
        odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_rob_rad_);

    }
    const nav_msgs::Odometry &getOdometry(){
        return odom_;
    }
    void track(const ros::Time &now, double dt, double vel_x, double vel_y, double vel_theta){
        // calculation from ROS odom publisher tutorial http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom, using now midpoint integration

        if(dt > 0){
            odom_.header.stamp = now;

            double vel_x_mid = (vel_x+odom_.twist.twist.linear.x)/2.0;
            double vel_y_mid = (vel_y+odom_.twist.twist.linear.y)/2.0;

            double sin_theta = sin(theta_rob_rad_);
            double cos_theta = cos(theta_rob_rad_);
            theta_rob_rad_ += vel_theta * dt;

            odom_.pose.pose.position.x += (vel_x_mid * cos_theta - vel_y_mid * sin_theta) * dt;
            odom_.pose.pose.position.y += (vel_x_mid * sin_theta + vel_y_mid * cos_theta) * dt;
            odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_rob_rad_);

            odom_.twist.twist.linear.x = vel_x;
            odom_.twist.twist.linear.y = vel_y;
            odom_.twist.twist.angular.z = vel_theta;
        }
    }
};


#endif
