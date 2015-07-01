/*
 * trajectory_interpolator.cpp
 *
 *  Created on: Jun 30, 2015
 *      Author: fxm-cm
 */


#include <cob_cartesian_controller/trajectory_interpolator/trajectory_interpolator.h>


void TrajectoryInterpolator::linear_interpolation(  std::vector <geometry_msgs::Pose>& poseVector,
                                                    geometry_msgs::Pose start, geometry_msgs::Pose end,
                                                    double VelMax, double AcclMax, std::string profile, bool justRotate)
{
    std::vector<double> linearPath, rollPath, pitchPath, yawPath;

    geometry_msgs::Pose pose;
    tf::Quaternion q;
    std::vector<double> pathMatrix[4];
    double start_roll, start_pitch, start_yaw;
    double end_roll, end_pitch, end_yaw;
    double Se = sqrt(pow((end.position.x-start.position.x), 2) + pow((end.position.y-start.position.y), 2) + pow((end.position.z-start.position.z), 2));

    // Convert Quaternions into RPY Angles for start and end pose
    q = tf::Quaternion(start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w);
    tf::Matrix3x3(q).getRPY(start_roll, start_pitch, start_yaw);

    q = tf::Quaternion(end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w);
    tf::Matrix3x3(q).getRPY(end_roll, end_pitch, end_yaw);

    // Calculate path length for the angular movement
    double Se_roll, Se_pitch, Se_yaw;
    Se_roll     = end_roll  - start_roll;
    Se_pitch    = end_pitch - start_pitch;
    Se_yaw      = end_yaw   - start_yaw;

//    ROS_INFO_STREAM("Length: " << std::endl << Se_roll << std::endl << Se_pitch << std::endl << Se_yaw << std::endl);


    // Calculate path for each Angle
    TPG_.calculateProfileForAngularMovements(pathMatrix, Se, Se_roll, Se_pitch, Se_yaw, start_roll, start_pitch, start_yaw,
                                             VelMax, AcclMax, profile, justRotate);

    linearPath  = pathMatrix[0];
    rollPath    = pathMatrix[1];
    pitchPath   = pathMatrix[2];
    yawPath     = pathMatrix[3];

    // Interpolate the linear path
    for(int i = 0 ; i < pathMatrix[0].size() ; i++)
    {
        if(!justRotate)
        {
            pose.position.x = start.position.x + linearPath.at(i) * (end.position.x-start.position.x)/Se;
            pose.position.y = start.position.y + linearPath.at(i) * (end.position.y-start.position.y)/Se;
            pose.position.z = start.position.z + linearPath.at(i) * (end.position.z-start.position.z)/Se;
        }
        else
        {
            pose.position.x = start.position.x;
            pose.position.y = start.position.y;
            pose.position.z = start.position.z;
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
}

void TrajectoryInterpolator::circular_interpolation(std::vector<geometry_msgs::Pose>& poseVector,
                                                    double M_x, double M_y, double M_z,
                                                    double M_roll, double M_pitch, double M_yaw,
                                                    double startAngle, double endAngle, double r, double VelMax, double AcclMax,
                                                    std::string profile)
{
    tf::Transform C, P, T;
    tf::Quaternion q;
    geometry_msgs::Pose pose, pos;
    std::vector<double> pathArray;

    // Convert RPY angles into [RAD]
    startAngle  = startAngle * M_PI/180;
    endAngle    = endAngle   * M_PI/180;
    M_roll      = M_roll     * M_PI/180;
    M_pitch     = M_pitch    * M_PI/180;
    M_yaw       = M_yaw      * M_PI/180;

    double Se = endAngle-startAngle;
    bool forward;

    if(Se < 0)
        forward = false;
    else
        forward = true;

    Se = std::fabs(Se);

    // Calculates the Path with Ramp - or Sinoidprofile
    TPG_.calculateProfile(pathArray, Se, VelMax, AcclMax, profile);

    // Define Center Pose
    C.setOrigin(tf::Vector3(M_x, M_y, M_z));
    q.setRPY(M_roll, M_pitch, M_yaw);
    C.setRotation(q);

    // Interpolate the circular path
    for(int i = 0 ; i < pathArray.size() ; i++)
    {
        // Rotate T
        T.setOrigin(tf::Vector3(cos(pathArray.at(i))*r, 0, sin(pathArray.at(i))*r));

        if(forward)
        {
            T.setOrigin(tf::Vector3(cos(pathArray.at(i))*r, 0, sin(pathArray.at(i))*r));
            q.setRPY(0,-pathArray.at(i),0);
        }
        else
        {
            T.setOrigin(tf::Vector3(cos(startAngle-pathArray.at(i))*r, 0, sin(startAngle-pathArray.at(i))*r));
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
    // Visualize center point
//    int marker4=0;
//    q.setRPY(M_roll+(M_PI/2),M_pitch,M_yaw);
//    C.setRotation(q);
//    showLevel(C,marker4,1.0,0,0,"level");
}
