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


#include <math.h>
#include <vector>

#include <cob_cartesian_controller/trajectory_interpolator/trajectory_interpolator.h>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_builder.h>

bool TrajectoryInterpolator::linearInterpolation(geometry_msgs::PoseArray& pose_array,
                                                 const cob_cartesian_controller::CartesianActionStruct as)
{
    this->trajectory_profile_generator_.reset(TrajectoryProfileBuilder::createProfile(as));

    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = root_frame_;

    tf::Quaternion q_start, q_end;

    std::vector<double> linear_path, angular_path, path;
    std::vector<double> path_matrix[2];
    geometry_msgs::Pose pose;

    double norm_factor;
    tf::quaternionMsgToTF(as.move_lin.start.orientation, q_start);
    tf::quaternionMsgToTF(as.move_lin.end.orientation, q_end);

    double Se_lin = sqrt(pow((as.move_lin.end.position.x - as.move_lin.start.position.x), 2) +
                         pow((as.move_lin.end.position.y - as.move_lin.start.position.y), 2) +
                         pow((as.move_lin.end.position.z - as.move_lin.start.position.z), 2));

    double Se_rot = q_start.angleShortestPath(q_end);

    if (!trajectory_profile_generator_->calculateProfile(path_matrix, Se_lin, Se_rot))
    {
        return false;
    }

    linear_path  = path_matrix[0];
    angular_path = path_matrix[1];

    if (fabs(linear_path.back()) > fabs(angular_path.back()))
    {
        path = linear_path;
    }
    else
    {
        path = angular_path;
    }
    norm_factor = 1/path.back();

    // Interpolate the linear path
    for (unsigned int i = 0; i < linear_path.size(); i++)
    {
        if (linear_path.back() == 0)
        {
            pose.position.x = as.move_lin.start.position.x;
            pose.position.y = as.move_lin.start.position.y;
            pose.position.z = as.move_lin.start.position.z;
        }
        else
        {
            pose.position.x = as.move_lin.start.position.x + linear_path.at(i) * (as.move_lin.end.position.x - as.move_lin.start.position.x) / linear_path.back();
            pose.position.y = as.move_lin.start.position.y + linear_path.at(i) * (as.move_lin.end.position.y - as.move_lin.start.position.y) / linear_path.back();
            pose.position.z = as.move_lin.start.position.z + linear_path.at(i) * (as.move_lin.end.position.z - as.move_lin.start.position.z) / linear_path.back();
        }

        tf::quaternionTFToMsg(q_start.slerp(q_end, path.at(i) * norm_factor), pose.orientation);
        pose_array.poses.push_back(pose);
    }
    return true;
}

bool TrajectoryInterpolator::circularInterpolation(geometry_msgs::PoseArray& pose_array,
                                                   const cob_cartesian_controller::CartesianActionStruct as)
{
     pose_array.header.stamp = ros::Time::now();
     pose_array.header.frame_id = root_frame_;

     tf::Quaternion q;
     tf::Transform C, P, T;

     std::vector<double> path_array;
     std::vector<double> path_matrix[2];

     geometry_msgs::Pose pose;

     double Se = as.move_circ.end_angle - as.move_circ.start_angle;

     bool forward;

     // Needed for the circle direction!
     if (Se < 0)
     {
         forward = false;
     }
     else
     {
         forward = true;
     }
     Se = std::fabs(Se);

     // Calculates the Path with RAMP or SINOID profile
     if (!this->trajectory_profile_generator_->calculateProfile(path_matrix, Se, 0))
     {
         return false;
     }

     path_array = path_matrix[0];
     // Define Center Pose
     C.setOrigin(tf::Vector3(as.move_circ.pose_center.position.x, as.move_circ.pose_center.position.y, as.move_circ.pose_center.position.z));
     tf::quaternionMsgToTF(as.move_circ.pose_center.orientation, q);
     C.setRotation(q);

     // Interpolate the circular path
     for (unsigned int i = 0; i < path_array.size(); i++)
     {
         // Rotate T
         T.setOrigin(tf::Vector3(cos(path_array.at(i)) * as.move_circ.radius, 0, sin(path_array.at(i)) * as.move_circ.radius));

         if (forward)
         {
             T.setOrigin(tf::Vector3(cos(path_array.at(i)) * as.move_circ.radius, 0, sin(path_array.at(i)) * as.move_circ.radius));
             q.setRPY(0, -path_array.at(i), 0);
         }
         else
         {
             T.setOrigin(tf::Vector3(cos(as.move_circ.start_angle - path_array.at(i)) * as.move_circ.radius, 0, sin(as.move_circ.start_angle - path_array.at(i)) * as.move_circ.radius));
             q.setRPY(0, path_array.at(i), 0);
         }

         T.setRotation(q);

         // Calculate TCP Position
         P = C * T;

         tf::pointTFToMsg(P.getOrigin(), pose.position);
         tf::quaternionTFToMsg(P.getRotation(), pose.orientation);

         pose_array.poses.push_back(pose);
     }

    return true;
}
