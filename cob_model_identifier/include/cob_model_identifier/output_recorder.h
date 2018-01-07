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


#ifndef OUTPUT_RECORDER_H
#define OUTPUT_RECORDER_H

#include <termios.h>
#include <signal.h>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include <boost/thread.hpp>


class OutputRecorder
{
public:
    bool initialize();
    void run();
    void stopRecording();
    void quit(int sig);

    void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    geometry_msgs::Pose getEndeffectorPose();

    double calculateLS(std::vector<double>* vec_out, std::vector<double>* vec_in, int model_order,
                       double& a1, double& a2, double& a3,
                       double& b1, double& b2, double& b3);
    void pseudoInverse(const Eigen::MatrixXd& matrix, Eigen::MatrixXd& matrix_inv, double tolerance);
    void euler(std::vector<double>* out, double in, double dt);
    void fillDataVectors(double x_dot_lin_in, double x_dot_lin_out,
                         double y_dot_lin_in, double y_dot_lin_out,
                         double z_dot_lin_in, double z_dot_lin_out,
                         double x_dot_rot_in, double x_dot_rot_out,
                         double y_dot_rot_in, double y_dot_rot_out,
                         double z_dot_rot_in, double z_dot_rot_out,
                         double x_lin_out, double y_lin_out, double z_lin_out, double x_rot_out, double y_rot_out, double z_rot_out);
    void stepResponsePlot(std::string file_name, std::vector<double>* in,
                          std::vector<double>* x_lin_out, std::vector<double>* y_lin_out, std::vector<double>* z_lin_out,
                          std::vector<double>* x_rot_out, std::vector<double>* y_rot_out, std::vector<double>* z_rot_out);
    void writeToMFile(std::string file_name, std::vector<double>* dot_in, std::vector<double>* dot_out, std::vector<double>* pos_out, std::vector<double>* dot_integrated);

private:
    ros::NodeHandle nh_;
    ros::Subscriber twist_sub_;
    ros::Subscriber jointstate_sub_;

    /// KDL Conversion
    KDL::Chain chain_;
    std::string output_file_path_;
    std::string chain_base_link_;
    std::string chain_tip_link_;
    KDL::JntArray last_q_;
    KDL::JntArray last_q_dot_;
    std::vector<std::string> joints_;
    KDL::ChainFkSolverVel_recursive* jntToCartSolver_vel_;
    unsigned int dof_;
    KDL::Vector vector_vel_, vector_rot_;

    /// Outputs
    // Velocity
    std::vector<double> x_dot_lin_vec_out_, y_dot_lin_vec_out_, z_dot_lin_vec_out_;
    std::vector<double> x_dot_rot_vec_out_, y_dot_rot_vec_out_, z_dot_rot_vec_out_;

    // Position
    std::vector<double> x_lin_vec_out_, y_lin_vec_out_, z_lin_vec_out_;
    std::vector<double> x_rot_vec_out_, y_rot_vec_out_, z_rot_vec_out_;
    double q_x_lin_out, q_y_lin_out, q_z_lin_out;

    /// Inputs
    std::vector<double> x_dot_lin_vec_in_, y_dot_lin_vec_in_, z_dot_lin_vec_in_;
    std::vector<double> x_dot_rot_vec_in_, y_dot_rot_vec_in_, z_dot_rot_vec_in_;

    double x_dot_lin_in_, y_dot_lin_in_, z_dot_lin_in_;
    double x_dot_rot_in_, y_dot_rot_in_, z_dot_rot_in_;

    /// Transform Listener
    tf::TransformListener listener_;

    /// Euler Integration
    double dt_;

    bool start_;
    bool finished_recording_;
    std::vector<double> time_vec_;

    /// For Keyboard commands
    char c;
    int kfd;
    struct termios cooked, raw;
};

#endif

