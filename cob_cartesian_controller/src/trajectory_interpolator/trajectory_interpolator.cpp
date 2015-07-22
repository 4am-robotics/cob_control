/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: care-o-bot
 * \note
 *   ROS stack name: cob_control
 * \note
 *   ROS package name: cob_cartesian_controller
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de / christoph.mark@gmail.com
 *
 * \date Date of creation: July, 2015
 *
 * \brief
 *   ...
 *
 ****************************************************************/

#include <cob_cartesian_controller/trajectory_interpolator/trajectory_interpolator.h>

bool TrajectoryInterpolator::linear_interpolation(  std::vector <geometry_msgs::Pose>& poseVector,
                                                    trajectory_action_move_lin& taml)
{
    std::vector<double> linearPath, rollPath, pitchPath, yawPath;

    geometry_msgs::Pose pose;
    tf::Quaternion q;
    std::vector<double> pathMatrix[4];
    double start_roll, start_pitch, start_yaw;
    double end_roll, end_pitch, end_yaw;
    double Se = sqrt(pow((taml.end.position.x - taml.start.position.x), 2) +
                     pow((taml.end.position.y - taml.start.position.y), 2) +
                     pow((taml.end.position.z - taml.start.position.z), 2));

    // Convert Quaternions into RPY Angles for start and end pose
    q = tf::Quaternion(taml.start.orientation.x,
                       taml.start.orientation.y,
                       taml.start.orientation.z,
                       taml.start.orientation.w);

    tf::Matrix3x3(q).getRPY(start_roll, start_pitch, start_yaw);

    q = tf::Quaternion(taml.end.orientation.x,
                       taml.end.orientation.y,
                       taml.end.orientation.z,
                       taml.end.orientation.w);

    tf::Matrix3x3(q).getRPY(end_roll, end_pitch, end_yaw);

    // Calculate path length for the angular movement
    double Se_roll, Se_pitch, Se_yaw;
    Se_roll     = end_roll  - start_roll;
    Se_pitch    = end_pitch - start_pitch;
    Se_yaw      = end_yaw   - start_yaw;

    // Calculate path for each Angle
    if(!TPG_.calculateProfileForAngularMovements(pathMatrix, Se, Se_roll, Se_pitch, Se_yaw, taml))
    {
        return false;
    }

    linearPath  = pathMatrix[0];
    rollPath    = pathMatrix[1];
    pitchPath   = pathMatrix[2];
    yawPath     = pathMatrix[3];

    // Interpolate the linear path
    for(int i = 0 ; i < pathMatrix[0].size() ; i++)
    {
        if(!taml.rotate_only)
        {
            pose.position.x = taml.start.position.x + linearPath.at(i) * (taml.end.position.x - taml.start.position.x) / Se;
            pose.position.y = taml.start.position.y + linearPath.at(i) * (taml.end.position.y - taml.start.position.y) / Se;
            pose.position.z = taml.start.position.z + linearPath.at(i) * (taml.end.position.z - taml.start.position.z) / Se;
        }
        else
        {
            pose.position.x = taml.start.position.x;
            pose.position.y = taml.start.position.y;
            pose.position.z = taml.start.position.z;
        }

        // Transform RPY to Quaternion
        q.setRPY(rollPath.at(i), pitchPath.at(i), yawPath.at(i));

        // Get Quaternion Values
        pose.orientation.x = q.getX();
        pose.orientation.y = q.getY();
        pose.orientation.z = q.getZ();
        pose.orientation.w = q.getW();
        poseVector.push_back(pose);
    }
    return true;
}

bool TrajectoryInterpolator::circular_interpolation(std::vector<geometry_msgs::Pose>& poseVector,
                                                    trajectory_action_move_circ &tamc)
{
    tf::Transform C, P, T;
    tf::Quaternion q;
    geometry_msgs::Pose pose, pos;
    std::vector<double> pathArray;

    // Convert RPY angles into [RAD]
    tamc.startAngle     *= M_PI/180;
    tamc.endAngle       *= M_PI/180;
    tamc.roll_center    *= M_PI/180;
    tamc.pitch_center   *= M_PI/180;
    tamc.yaw_center     *= M_PI/180;

    double Se = tamc.endAngle-tamc.startAngle;
    bool forward;

    if(Se < 0)
        forward = false;
    else
        forward = true;

    Se = std::fabs(Se);

    // Calculates the Path with Ramp - or Sinoidprofile
    if(!TPG_.calculateProfile(pathArray, Se, tamc.vel, tamc.accl, tamc.profile))
    {
        return false;
    }

    // Define Center Pose
    C.setOrigin(tf::Vector3(tamc.x_center, tamc.y_center, tamc.z_center));
    q.setRPY(tamc.roll_center, tamc.pitch_center, tamc.yaw_center);
    C.setRotation(q);

    // Interpolate the circular path
    for(int i = 0 ; i < pathArray.size() ; i++)
    {
        // Rotate T
        T.setOrigin(tf::Vector3(cos(pathArray.at(i)) * tamc.radius, 0, sin(pathArray.at(i)) * tamc.radius));

        if(forward)
        {
            T.setOrigin(tf::Vector3(cos(pathArray.at(i)) * tamc.radius, 0, sin(pathArray.at(i)) * tamc.radius));
            q.setRPY(0,-pathArray.at(i),0);
        }
        else
        {
            T.setOrigin(tf::Vector3(cos(tamc.startAngle-pathArray.at(i)) * tamc.radius, 0, sin(tamc.startAngle-pathArray.at(i)) * tamc.radius));
            q.setRPY(0, pathArray.at(i), 0);
        }

        T.setRotation(q);

        // Calculate TCP Position
        P = C * T;

        // Fill the Pose
        pose.position.x = P.getOrigin().x();
        pose.position.y = P.getOrigin().y();
        pose.position.z = P.getOrigin().z();

        pose.orientation.x = P.getRotation()[0];
        pose.orientation.y = P.getRotation()[1];
        pose.orientation.z = P.getRotation()[2];
        pose.orientation.w = P.getRotation()[3];

        // Put the pose into the pose Vector
        poseVector.push_back(pose);
    }
    return true;
}
