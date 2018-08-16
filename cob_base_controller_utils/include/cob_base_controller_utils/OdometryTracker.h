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

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#ifndef COB_ODOMETRY_TRACKER_H
#define COB_ODOMETRY_TRACKER_H

class OdometryTracker{
    nav_msgs::Odometry odom_;
    double theta_rob_rad_;
public:
    OdometryTracker(const std::string &from = "odom", const std::string &to = "base_footprint" , double cov_pose = 0.1, double cov_twist = 0.1) {
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
