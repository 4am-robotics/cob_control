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
#include <vector>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <cob_model_identifier/output_recorder.h>

#include <boost/thread.hpp>
#include <tinyxml.h>


bool OutputRecorder::initialize()
{
    ros::NodeHandle nh_twist("twist_controller");
    ros::NodeHandle nh_identifier("model_identifier");

    // JointNames
    if (!nh_.getParam("joint_names", joints_))
    {
        ROS_ERROR("Parameter 'joint_names' not set");
        return false;
    }
    dof_ = joints_.size();

    if (!nh_twist.getParam("chain_base_link", chain_base_link_))
    {
        ROS_ERROR("Parameter 'chain_base_link' not set");
        return false;
    }
    if (!nh_twist.getParam("chain_tip_link", chain_tip_link_))
    {
        ROS_ERROR("Parameter 'chain_tip_link' not set");
        return false;
    }

    /// parse robot_description and generate KDL chains
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromParam("/robot_description", my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    my_tree.getChain(chain_base_link_, chain_tip_link_, chain_);
    if (chain_.getNrOfJoints() == 0)
    {
        ROS_ERROR("Failed to initialize kinematic chain");
        return false;
    }

    // OutputFilePath
    if (nh_identifier.hasParam("output_file_path"))
    {
        nh_identifier.getParam("output_file_path", output_file_path_);
    }
    else
    {
        output_file_path_ = "/tmp/model_identifier/";
        ROS_ERROR("No parameter 'output_file_path'! Using default %s", output_file_path_.c_str());
    }
    ROS_WARN("'output_file_path'! %s", output_file_path_.c_str());

    /// initialize variables and current joint values and velocities
    last_q_ = KDL::JntArray(chain_.getNrOfJoints());
    last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());

    jointstate_sub_ = nh_.subscribe("joint_states", 1, &OutputRecorder::jointstateCallback, this);
    twist_sub_ = nh_twist.subscribe("command_twist_stamped", 1, &OutputRecorder::twistCallback, this);

    start_ = false;
    finished_recording_ = false;

    ROS_INFO("...initialized!");
    return true;
}

void OutputRecorder::run()
{
    tf::Transform trans;
    double x_dot_lin_in, y_dot_lin_in, z_dot_lin_in;
    double x_dot_rot_in, y_dot_rot_in, z_dot_rot_in;
    double x_dot_lin_out, y_dot_lin_out, z_dot_lin_out;
    double x_dot_rot_out, y_dot_rot_out, z_dot_rot_out;
    double x_lin_start, y_lin_start, z_lin_start;
    double x_rot_start, y_rot_start, z_rot_start;

    std::vector<double> x_dot_lin_integrated, y_dot_lin_integrated, z_dot_lin_integrated;
    std::vector<double> x_dot_rot_integrated, y_dot_rot_integrated, z_dot_rot_integrated;
    geometry_msgs::Pose q_soll, q_ist;

    int iterations = 0;
    ros::Rate r(100.0);

    ROS_INFO("Waiting for Twist callback");

    while (!start_ && ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    boost::thread start_thread;
    start_thread = boost::thread(boost::bind(&OutputRecorder::stopRecording, this));
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ROS_INFO("Start recording \n Enter any key to stop it.");

    ros::Time time = ros::Time::now();
    ros::Time last_update_time = time;
    ros::Duration period = time - last_update_time;

    q_ist = getEndeffectorPose();
    x_lin_start = q_ist.position.x;
    y_lin_start = q_ist.position.y;
    z_lin_start = q_ist.position.z;

    double roll, pitch, yaw;
    tf::Quaternion q = tf::Quaternion(q_ist.orientation.x, q_ist.orientation.y, q_ist.orientation.z, q_ist.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    while (!finished_recording_)
    {
        // start_thread.join();
        start_thread.interrupt();
        time = ros::Time::now();
        period = time - last_update_time;
        q_ist = getEndeffectorPose();

        /// Ist Position
        q_x_lin_out = q_ist.position.x;
        q_y_lin_out = q_ist.position.y;
        q_z_lin_out = q_ist.position.z;
        q = tf::Quaternion(q_ist.orientation.x, q_ist.orientation.y, q_ist.orientation.z, q_ist.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        /// Sollgeschwindigkeit
        x_dot_lin_in = x_dot_lin_in_;
        y_dot_lin_in = y_dot_lin_in_;
        z_dot_lin_in = z_dot_lin_in_;
        x_dot_rot_in = x_dot_rot_in_;
        y_dot_rot_in = y_dot_rot_in_;
        z_dot_rot_in = z_dot_rot_in_;

        fillDataVectors(x_dot_lin_in, vector_vel_.x(),
                        y_dot_lin_in, vector_vel_.y(),
                        z_dot_lin_in, vector_vel_.z(),
                        x_dot_rot_in, vector_rot_.x(),
                        y_dot_rot_in, vector_rot_.y(),
                        z_dot_rot_in, vector_rot_.z(),
                        q_x_lin_out, q_y_lin_out, q_z_lin_out, roll, pitch, yaw);

        euler(&x_dot_lin_integrated, vector_vel_.x(), period.toSec());
        euler(&y_dot_lin_integrated, vector_vel_.y(), period.toSec());
        euler(&z_dot_lin_integrated, vector_vel_.z(), period.toSec());

        euler(&x_dot_rot_integrated, vector_rot_.x(), period.toSec());
        euler(&y_dot_rot_integrated, vector_rot_.y(), period.toSec());
        euler(&z_dot_rot_integrated, vector_rot_.z(), period.toSec());

        if (iterations < 1)
        {
            time_vec_.push_back(period.toSec());
        }
        else
        {
            time_vec_.push_back(time_vec_.at(iterations)+period.toSec());
        }
        dt_ += period.toSec();
        iterations++;

        last_update_time = time;
        ros::spinOnce();
        r.sleep();
    }
    tcsetattr(kfd, TCSANOW, &cooked);

    ROS_INFO("Stopped recording... preparing output for octave plot ");

    /// Generate Octave Files
    writeToMFile("x_linear", &x_dot_lin_vec_in_, &x_dot_lin_vec_out_, &x_lin_vec_out_, &x_dot_lin_integrated);
    writeToMFile("y_linear", &y_dot_lin_vec_in_, &y_dot_lin_vec_out_, &y_lin_vec_out_, &y_dot_lin_integrated);
    writeToMFile("z_linear", &z_dot_lin_vec_in_, &z_dot_lin_vec_out_, &z_lin_vec_out_, &z_dot_lin_integrated);

    writeToMFile("x_angular", &x_dot_rot_vec_in_, &x_dot_rot_vec_out_, &x_rot_vec_out_, &x_dot_rot_integrated);
    writeToMFile("y_angular", &y_dot_rot_vec_in_, &y_dot_rot_vec_out_, &y_rot_vec_out_, &y_dot_rot_integrated);
    writeToMFile("z_angular", &z_dot_rot_vec_in_, &z_dot_rot_vec_out_, &z_rot_vec_out_, &z_dot_rot_integrated);

    /// Velocity Stepresponse
    stepResponsePlot("x_linear_dot_step", &x_dot_lin_vec_in_, &x_dot_lin_vec_out_, &y_dot_lin_vec_out_, &z_dot_lin_vec_out_, &x_dot_rot_vec_out_, &y_dot_rot_vec_out_, &z_dot_rot_vec_out_);
    stepResponsePlot("y_linear_dot_step", &y_dot_lin_vec_in_, &x_dot_lin_vec_out_, &y_dot_lin_vec_out_, &z_dot_lin_vec_out_, &x_dot_rot_vec_out_, &y_dot_rot_vec_out_, &z_dot_rot_vec_out_);
    stepResponsePlot("z_linear_dot_step", &z_dot_lin_vec_in_, &x_dot_lin_vec_out_, &y_dot_lin_vec_out_, &z_dot_lin_vec_out_, &x_dot_rot_vec_out_, &y_dot_rot_vec_out_, &z_dot_rot_vec_out_);
    stepResponsePlot("x_angular_dot_step", &x_dot_rot_vec_in_, &x_dot_lin_vec_out_, &y_dot_lin_vec_out_, &z_dot_lin_vec_out_, &x_dot_rot_vec_out_, &y_dot_rot_vec_out_, &z_dot_rot_vec_out_);
    stepResponsePlot("y_angular_dot_step", &y_dot_rot_vec_in_, &x_dot_lin_vec_out_, &y_dot_lin_vec_out_, &z_dot_lin_vec_out_, &x_dot_rot_vec_out_, &y_dot_rot_vec_out_, &z_dot_rot_vec_out_);
    stepResponsePlot("z_angular_dot_step", &z_dot_rot_vec_in_, &x_dot_lin_vec_out_, &y_dot_lin_vec_out_, &z_dot_lin_vec_out_, &x_dot_rot_vec_out_, &y_dot_rot_vec_out_, &z_dot_rot_vec_out_);

    /// Position Stepresponse
    stepResponsePlot("x_linear_step", &x_dot_lin_vec_in_, &x_dot_lin_integrated, &y_dot_lin_integrated, &z_dot_lin_integrated, &x_dot_rot_integrated, &y_dot_rot_integrated, &z_dot_rot_integrated);
    stepResponsePlot("y_linear_step", &y_dot_lin_vec_in_, &x_dot_lin_integrated, &y_dot_lin_integrated, &z_dot_lin_integrated, &x_dot_rot_integrated, &y_dot_rot_integrated, &z_dot_rot_integrated);
    stepResponsePlot("z_linear_step", &z_dot_lin_vec_in_, &x_dot_lin_integrated, &y_dot_lin_integrated, &z_dot_lin_integrated, &x_dot_rot_integrated, &y_dot_rot_integrated, &z_dot_rot_integrated);
    stepResponsePlot("x_angular_step", &x_dot_rot_vec_in_, &x_dot_lin_integrated, &y_dot_lin_integrated, &z_dot_lin_integrated, &x_dot_rot_integrated, &y_dot_rot_integrated, &z_dot_rot_integrated);
    stepResponsePlot("y_angular_step", &y_dot_rot_vec_in_, &x_dot_lin_integrated, &y_dot_lin_integrated, &z_dot_lin_integrated, &x_dot_rot_integrated, &y_dot_rot_integrated, &z_dot_rot_integrated);
    stepResponsePlot("z_angular_step", &z_dot_rot_vec_in_, &x_dot_lin_integrated, &y_dot_lin_integrated, &z_dot_lin_integrated, &x_dot_rot_integrated, &y_dot_rot_integrated, &z_dot_rot_integrated);
}

void OutputRecorder::stopRecording()
{
    c = 0x0;
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    while (ros::ok())
    {
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        if (c == 0x61)
        {
            finished_recording_ = true;
            break;
        }
    }
}

void OutputRecorder::quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

void OutputRecorder::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    KDL::JntArray q_temp = last_q_;
    KDL::JntArray q_dot_temp = last_q_dot_;
    int count = 0;

    for (unsigned int j = 0; j < dof_; j++)
    {
        for (unsigned int i = 0; i < msg->name.size(); i++)
        {
            if (strcmp(msg->name[i].c_str(), joints_[j].c_str()) == 0)
            {
                q_temp(j) = msg->position[i];
                q_dot_temp(j) = msg->velocity[i];
                count++;
                break;
            }
        }
    }

    if (count == joints_.size())
    {
        KDL::FrameVel FrameVel;
        last_q_ = q_temp;
        last_q_dot_ = q_dot_temp;
        KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(last_q_, last_q_dot_);

        jntToCartSolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);
        int ret = jntToCartSolver_vel_->JntToCart(jntArrayVel, FrameVel, -1);

        if (ret >= 0)
        {
            KDL::Twist twist = FrameVel.GetTwist();
            vector_vel_ = twist.vel;
            vector_rot_ = twist.rot;
            // ROS_INFO("ist_vel: %f %f %f",twist.vel.x(), twist.vel.y(), twist.vel.z());
        }
        else
        {
            ROS_WARN("ChainFkSolverVel failed!");
        }
    }
    else
    {
        ROS_ERROR("jointstateCallback: received unexpected 'joint_states'");
    }
}

void OutputRecorder::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    start_ = true;
    x_dot_lin_in_ = msg->twist.linear.x;
    y_dot_lin_in_ = msg->twist.linear.y;
    z_dot_lin_in_ = msg->twist.linear.z;

    x_dot_rot_in_ = msg->twist.angular.x;
    y_dot_rot_in_ = msg->twist.angular.y;
    z_dot_rot_in_ = msg->twist.angular.z;
}

geometry_msgs::Pose OutputRecorder::getEndeffectorPose()
{
    ros::Time::now();
    geometry_msgs::Pose pos;
    tf::StampedTransform stampedTransform;

    // Get transformation
    try
    {
        listener_.lookupTransform(chain_base_link_, chain_tip_link_, ros::Time(0), stampedTransform);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    pos.position.x = stampedTransform.getOrigin().x();
    pos.position.y = stampedTransform.getOrigin().y();
    pos.position.z = stampedTransform.getOrigin().z();
    pos.orientation.x = stampedTransform.getRotation()[0];
    pos.orientation.y = stampedTransform.getRotation()[1];
    pos.orientation.z = stampedTransform.getRotation()[2];
    pos.orientation.w = stampedTransform.getRotation()[3];

    return pos;
}

double OutputRecorder::calculateLS(std::vector<double>* vec_out, std::vector<double>* vec_in, int model_order, double& a1, double& a2, double& a3, double& b1, double& b2, double& b3)
{
    double err = 0;
    Eigen::MatrixXd F(vec_out->size()-model_order, model_order*2);
    Eigen::VectorXd y(vec_out->size()-model_order);
    Eigen::MatrixXd F_pinv;
    Eigen::MatrixXd theta;
    std::vector<double> errorVector;
    int d = 0;

    /// System 1. Ordnung
    if (model_order == 1)
    {
        for (int i = 0; i < vec_out->size()-model_order; i++)
        {
            if (i < d)
            {
                F(i, 0) = 0;
                y(i) = 0;
            }
            else
            {
                F(i, 0) = -1 * vec_out->at(i);
                y(i) = vec_out->at(i+1);
            }
            F(i, 1) = vec_in->at(i);
        }
    }

    /// System 2. Ordnung
    if (model_order == 2)
    {
        for (int i = 0; i < vec_out->size()-model_order; i++)
        {
            if (i < d)
            {
                F(i, 0) = 0;
                F(i, 1) = 0;
                y(i) = 0;
            }
            else
            {
                F(i, 0) = -1 * vec_out->at(i+1);
                F(i, 1) = -1 * vec_out->at(i);
                y(i) = vec_out->at(i+2);
            }
            F(i, 2) = vec_in->at(i+1);
            F(i, 3) = vec_in->at(i);
        }
    }

    /// System 3. Ordnung
    if (model_order == 3)
    {
        for (int i = 0; i < vec_out->size()-model_order; i++)
        {
            if (i < d)
            {
                F(i, 0) = 0;
                F(i, 1) = 0;
                F(i, 2) = 0;
                y(i) = 0;
            }
            else
            {
                F(i, 0) = -1 * vec_out->at(i+2);
                F(i, 1) = -1 * vec_out->at(i+1);
                F(i, 2) = -1 * vec_out->at(i);
                y(i) = vec_out->at(i+3);
            }
            F(i, 3) = vec_in->at(i+2);
            F(i, 4) = vec_in->at(i+1);
            F(i, 5) = vec_in->at(i);
        }
    }

    /// 'double' machine precision http://freemat.sourceforge.net/help/constants_eps.html
    pseudoInverse(F, F_pinv, (2.2204*pow(10, -16)));
    theta = F_pinv * y;
    Eigen::MatrixXd e = y - F * theta;
    Eigen::MatrixXd e_squared = e.transpose() * e;
    err = e_squared(0, 0);

    errorVector.push_back(err);

    a1 = a2 = a3 = b1 = b2 = b3 = 0;
    if (model_order == 1)
    {
        a1 = theta(0, 0);
        b1 = theta(1, 0);
    }

    if (model_order == 2)
    {
        a1 = theta(0, 0);
        a2 = theta(1, 0);
        b1 = theta(2, 0);
        b2 = theta(3, 0);
    }

    if (model_order == 3)
    {
        a1 = theta(0, 0);
        a2 = theta(1, 0);
        a3 = theta(2, 0);
        b1 = theta(3, 0);
        b2 = theta(4, 0);
        b3 = theta(5, 0);
    }

    std::cout << "\n\nTheta: \n" << theta << std::endl;
    std::cout << "\n Squared Error: \n" << err << std::endl;

    return err;
}

void OutputRecorder::pseudoInverse(const Eigen::MatrixXd& matrix, Eigen::MatrixXd& matrix_inv, double tolerance)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svdOfM(matrix, Eigen::ComputeThinU  | Eigen::ComputeThinV);
    const Eigen::MatrixXd U = svdOfM.matrixU();
    const Eigen::MatrixXd V = svdOfM.matrixV();
    const Eigen::VectorXd S = svdOfM.singularValues();
    Eigen::VectorXd Sinv = S;

    double maxsv = 0;
    for (std::size_t i = 0; i < S.rows(); ++i)
    {
        if (fabs(S(i)) > maxsv)
        {
            maxsv = fabs(S(i));
        }
    }

    for (std::size_t i = 0; i < S.rows(); ++i)
    {
        Sinv(i) = ((S(i)< 0.0001)?0:1/S(i));
    }

    matrix_inv = V * Sinv.asDiagonal() * U.transpose();
}

void OutputRecorder::euler(std::vector<double>* out, double in, double dt)
{
    if (out->size() == 0)
    {
        out->push_back(in*dt);
    }
    else
    {
        out->push_back(out->at(out->size()-1) + in*dt);
    }
}

void OutputRecorder::fillDataVectors(double x_dot_lin_in, double x_dot_lin_out,
                                     double y_dot_lin_in, double y_dot_lin_out,
                                     double z_dot_lin_in, double z_dot_lin_out,
                                     double x_dot_rot_in, double x_dot_rot_out,
                                     double y_dot_rot_in, double y_dot_rot_out,
                                     double z_dot_rot_in, double z_dot_rot_out,
                                     double x_lin_out, double y_lin_out, double z_lin_out, double x_rot_out, double y_rot_out, double z_rot_out)
{
    /// lin velocity
    x_dot_lin_vec_in_.push_back(x_dot_lin_in);
    x_dot_lin_vec_out_.push_back(x_dot_lin_out);
    y_dot_lin_vec_in_.push_back(y_dot_lin_in);
    y_dot_lin_vec_out_.push_back(y_dot_lin_out);
    z_dot_lin_vec_in_.push_back(z_dot_lin_in);
    z_dot_lin_vec_out_.push_back(z_dot_lin_out);

    /// rot velocity
    x_dot_rot_vec_in_.push_back(x_dot_rot_in);
    x_dot_rot_vec_out_.push_back(x_dot_rot_out);
    y_dot_rot_vec_in_.push_back(y_dot_rot_in);
    y_dot_rot_vec_out_.push_back(y_dot_rot_out);
    z_dot_rot_vec_in_.push_back(z_dot_rot_in);
    z_dot_rot_vec_out_.push_back(z_dot_rot_out);

    /// lin position
    x_lin_vec_out_.push_back(x_lin_out);
    y_lin_vec_out_.push_back(y_lin_out);
    z_lin_vec_out_.push_back(z_lin_out);

    /// rot position
    x_rot_vec_out_.push_back(x_rot_out);
    y_rot_vec_out_.push_back(y_rot_out);
    z_rot_vec_out_.push_back(z_rot_out);
}

void OutputRecorder::stepResponsePlot(std::string file_name, std::vector<double>* in, std::vector<double>* x_lin_out, std::vector<double>* y_lin_out, std::vector<double>* z_lin_out, std::vector<double>* x_rot_out, std::vector<double>* y_rot_out, std::vector<double>* z_rot_out)
{
    std::ofstream myfile;
    std::string name;

    name = output_file_path_ + file_name + ".m";
    ROS_INFO("Writing results to: %s", name.c_str());
    const char* charPath = name.c_str();

    myfile.open(charPath);
    myfile << "clear all;close all;\n\n";

    /// Get the vectors and write them to .m file-----------------------
    myfile << file_name << "_in = [" << std::endl;
    for (int i = 0; i < in->size()-1; i++)
    {
        myfile << in->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << file_name << "_x_lin_out = [" << std::endl;
    for (int i = 0; i < x_lin_out->size()-1; i++)
    {
        myfile << x_lin_out->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << file_name << "_y_lin_out = [" << std::endl;
    for (int i = 0; i < y_lin_out->size()-1; i++)
    {
        myfile << y_lin_out->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << file_name << "_z_lin_out = [" << std::endl;
    for (int i = 0; i < z_lin_out->size()-1; i++)
    {
        myfile << z_lin_out->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << file_name << "_x_rot_out = [" << std::endl;
    for (int i = 0; i < x_rot_out->size()-1; i++)
    {
        myfile << x_rot_out->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << file_name << "_y_rot_out = [" << std::endl;
    for (int i = 0; i < y_rot_out->size()-1; i++)
    {
        myfile << y_rot_out->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << file_name << "_z_rot_out = [" << std::endl;
    for (int i = 0; i < z_rot_out->size()-1; i++)
    {
        myfile << z_rot_out->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << "k = size(" <<file_name << "_in);" << std::endl;
    myfile << "t = linspace(0,k(1)*" << fabs(dt_) << ",size(" <<file_name << "_in));" << std::endl;

    myfile << "figure" << std::endl;
    myfile << "plot(t," << file_name << "_in,t," << file_name << "_x_lin_out,t," << file_name << "_y_lin_out,t," << file_name << "_z_lin_out,t," << file_name << "_x_rot_out,t," << file_name << "_y_rot_out,t," << file_name << "_z_rot_out)" << std::endl;
    myfile << "c=legend('Input','x_lin_out','y_lin_out','z_lin_out','x_rot_out','y_rot_out','z_rot_out','Location','NorthEastOutside'); \n set(c,'Interpreter','none');" << std::endl;
    myfile << "grid" << std::endl;

    myfile.close();
}


void OutputRecorder::writeToMFile(std::string file_name, std::vector<double>* dot_in, std::vector<double>* dot_out, std::vector<double>* pos_out, std::vector<double>* dot_integrated)
{
    std::ofstream myfile;
    std::string name;
    std::vector <double> errVec;
    double a1, a2, a3, b1, b2, b3 = 0.0;
    std::ostringstream a1_str, a2_str, a3_str, b1_str, b2_str, b3_str;

    name = output_file_path_ + file_name + ".m";
    ROS_INFO("Writing results to: %s", name.c_str());
    const char* charPath = name.c_str();

    myfile.open(charPath);

    myfile << "clear all;close all;\n\n";

    /// Get the vectors and write them to .m file
    myfile << file_name << "_dot_in = [" << std::endl;
    for (int i = 0; i < dot_in->size()-1; i++)
    {
        myfile << dot_in->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << file_name << "_dot_out = [" << std::endl;
    for (int i = 0; i < dot_out->size()-1; i++)
    {
        myfile << dot_out->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << file_name << "_pos_out = [" << std::endl;
    for (int i = 0; i < pos_out->size()-1; i++)
    {
        myfile << pos_out->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << file_name << "_dot_integrated = [" << std::endl;
    for (int i=0; i < dot_integrated->size()-1; i++)
    {
        myfile << dot_integrated->at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;

    myfile << "t = [" << std::endl;
    for (int i = 0; i < time_vec_.size()-1; i++)
    {
        myfile << time_vec_.at(i) <<std::endl;
    }
    myfile << "]; \n" << std::endl;
    ///-----------------------------------------------------------------

    /// Generate the time vector based on average sample time
    myfile << "k = size(" <<file_name << "_dot_in);" << std::endl;
    myfile << "t = linspace(0,k(1)*" << fabs(dt_) << ",size(" <<file_name << "_dot_in));" << std::endl;
    ///-----------------------------------------------------------------

    /// Generate Velocity Models----------------------------------------
    myfile << "s = tf('s'); z=tf('z',1/50);" << std::endl;

    errVec.clear();

    /// 1. Order
    a1_str.str(""); a2_str.str(""); a3_str.str(""); b1_str.str(""); b2_str.str(""); b3_str.str("");
    a1_str.clear(); a2_str.clear(); a3_str.clear(); b1_str.clear(); b2_str.clear(); b3_str.clear();

    errVec.push_back(calculateLS(dot_out, dot_in, 1, a1, a2, a3, b1, b2, b3));

    a1_str << a1;
    b1_str << b1;

    std::string Gz1 = "Gz_" + file_name + "1=" + b1_str.str() + "*z^-1/(1" + a1_str.str() + "*z^-1);";
    while (Gz1.find("+-") != std::string::npos)
    {
        Gz1.replace(Gz1.find("+-"), 2, "-");
    }

    myfile << Gz1;

    /// 2. Order
    a1_str.str(""); a2_str.str(""); a3_str.str(""); b1_str.str(""); b2_str.str(""); b3_str.str("");
    a1_str.clear(); a2_str.clear(); a3_str.clear(); b1_str.clear(); b2_str.clear(); b3_str.clear();

    errVec.push_back(calculateLS(dot_out, dot_in, 2, a1, a2, a3, b1, b2, b3));

    a1_str << a1;
    a2_str << a2;

    b1_str << b1;
    b2_str << b2;


    std::string Gz2 = "Gz_" + file_name + "2=(" + b1_str.str() + "*z^-1 +" + b2_str.str() + "*z^-2)/(1 +" + a1_str.str() + "*z^-1 +" + a2_str.str() + "*z^-2);";
    while (Gz2.find("+-") != std::string::npos)
    {
        Gz2.replace(Gz2.find("+-"), 2, "-");
    }

    myfile << Gz2;


    /// 3. Order
    a1_str.str(""); a2_str.str(""); a3_str.str(""); b1_str.str(""); b2_str.str(""); b3_str.str("");
    a1_str.clear(); a2_str.clear(); a3_str.clear(); b1_str.clear(); b2_str.clear(); b3_str.clear();

    errVec.push_back(calculateLS(dot_out, dot_in, 3, a1, a2, a3, b1, b2, b3));

    a1_str << a1;
    a2_str << a2;
    a3_str << a3;
    b1_str << b1;
    b2_str << b2;
    b3_str << b3;

    std::string Gz3 = "Gz_" + file_name + "3=(" + b1_str.str() + "*z^-1 +" + b2_str.str() + "*z^-2 +" + b3_str.str() + "*z^-3)/(1 +" + a1_str.str() + "*z^-1 +" + a2_str.str() + "*z^-2 +" + a3_str.str() + "*z^-3);";
    while (Gz3.find("+-") != std::string::npos)
    {
        Gz3.replace(Gz3.find("+-"), 2, "-");
    }
    myfile << Gz3;


    /// Errorplot to identify model order-------------------------------
    myfile << "e =[" << errVec.at(0) << "," << errVec.at(1) << ","  << errVec.at(2) << "];" << std::endl;
    myfile << "figure; semilogy(e); grid;" << std::endl;

    double step = 0.0;
    for (int i = 0; i < dot_in->size()-1; i++)
    {
        step+=dot_in->at(i);
    }
    step/=(dot_in->size()-1);
    std::cout << "step = " << step <<std::endl;

    int length = static_cast<int>(pos_out->size()*2/3) - static_cast<int>(pos_out->size()/3);
    double Ki = (pos_out->at(static_cast<int>(pos_out->size()*2/3)) - pos_out->at(static_cast<int>(pos_out->size()/3)))/(length * dt_) / step;

    ROS_INFO("length: %i    dY: %f   Ki: %f", length, (pos_out->at(static_cast<int>(pos_out->size()*2/3)) - pos_out->at(static_cast<int>(pos_out->size()/3))), Ki);

    /// First order
    myfile << "Gz_" << file_name << "1 = minreal(Gz_" << file_name << "1);" << std::endl;
    myfile << "Gs_" << file_name << "1=d2c(Gz_" << file_name << "1,'matched');" << std::endl;
    // myfile << "Gs_" << file_name << "_integrated1 = Gs_" << file_name << "1*(" << Ki << "/s);" << std::endl;
    myfile << "Gs_" << file_name << "_integrated1 = Gs_" << file_name << "1* (1/s);" << std::endl;
    myfile << "Gs_" << file_name << "_out1 = lsim(Gs_" << file_name << "1," << file_name << "_dot_in,t);" << std::endl;
    myfile << "Gs_" << file_name << "_out_integrated1 = lsim(Gs_" << file_name << "_integrated1," << file_name << "_dot_in,t);" << std::endl;

    // Plot
    myfile << "figure" << std::endl;
    myfile << "subplot(2,1,1);" << std::endl;
    myfile << "plot(t," << file_name << "_dot_in,t," << file_name << "_dot_out,t,Gs_" << file_name << "_out1)" << std::endl;
    myfile << "c=legend('Velocity Input','Velocity Systemresponse','PT1 Modelresponse','Location','NorthEastOutside'); \n set(c,'Interpreter','none');" << std::endl;
    myfile << "title('" << file_name << " Velocity Stepresponse','interpreter','none')"  << std::endl;
    myfile << "grid" << std::endl;
    myfile << "subplot(2,1,2);" << std::endl;
    myfile << "plot(t," << file_name << "_dot_in,t," << file_name << "_dot_integrated,t,Gs_" << file_name << "_out_integrated1)" << std::endl;
    myfile << "c=legend('Velocity Input','Position Systemresponse','IT1 Modelresponse','Location','NorthEastOutside'); \n set(c,'Interpreter','none');" << std::endl;
    myfile << "title('" << file_name << " Position Stepresponse','interpreter','none')"  << std::endl;
    myfile << "grid" << std::endl;

    /// Second order
    myfile << "Gz_" << file_name << "2 = minreal(Gz_" << file_name << "2);"<< std::endl;
    myfile << "Gs_" << file_name << "2=d2c(Gz_" << file_name << "2,'matched');" << std::endl;
    // myfile << "Gs_" << file_name << "_integrated2 = Gs_" << file_name << "2*(" << Ki << "/s);" << std::endl;
    myfile << "Gs_" << file_name << "_integrated2 = Gs_" << file_name << "2*(1/s);" << std::endl;
    myfile << "Gs_" << file_name << "_out2 = lsim(Gs_" << file_name << "2," << file_name << "_dot_in,t);" << std::endl;
    myfile << "Gs_" << file_name << "_out_integrated2 = lsim(Gs_" << file_name << "_integrated2," << file_name << "_dot_in,t);" << std::endl;

    // Plot
    myfile << "figure" << std::endl;
    myfile << "subplot(2,1,1);" << std::endl;
    myfile << "plot(t," << file_name << "_dot_in,t," << file_name << "_dot_out,t,Gs_" << file_name << "_out2)" << std::endl;
    myfile << "c=legend('Velocity Input','Velocity Systemresponse','PDT2 Modelresponse','Location','NorthEastOutside'); \n set(c,'Interpreter','none');" << std::endl;
    myfile << "title('" << file_name << " Velocity Stepresponse','interpreter','none')"  << std::endl;
    myfile << "grid" << std::endl;
    myfile << "subplot(2,1,2);" << std::endl;
    myfile << "plot(t," << file_name << "_dot_in,t," << file_name << "_dot_integrated,t,Gs_" << file_name << "_out_integrated2)" << std::endl;
    myfile << "c=legend('Velocity Input','Position Systemresponse','PIDT2 Modelresponse','Location','NorthEastOutside'); \n set(c,'Interpreter','none');" << std::endl;
    myfile << "title('" << file_name << " Position Stepresponse','interpreter','none')"  << std::endl;
    myfile << "grid" << std::endl;

    /// Third order
    myfile << "Gz_" << file_name << "3 = minreal(Gz_" << file_name << "3);" << std::endl;
    myfile << "Gs_" << file_name << "3=d2c(Gz_" << file_name << "3,'matched');" << std::endl;
    // myfile << "Gs_" << file_name << "_integrated3 = Gs_" << file_name << "3*(" << Ki << "/s);" << std::endl;
    myfile << "Gs_" << file_name << "_integrated3 = Gs_" << file_name << "3*(1/s);" << std::endl;
    myfile << "Gs_" << file_name << "_out3 = lsim(Gs_" << file_name << "3," << file_name << "_dot_in,t);" << std::endl;
    myfile << "Gs_" << file_name << "_out_integrated3 = lsim(Gs_" << file_name << "_integrated3," << file_name << "_dot_in,t);" << std::endl;

    // Plot
    myfile << "figure" << std::endl;
    myfile << "subplot(2,1,1);" << std::endl;
    myfile << "plot(t," << file_name << "_dot_in,t," << file_name << "_dot_out,t,Gs_" << file_name << "_out3)" << std::endl;
    myfile << "c=legend('Velocity Input','Velocity Systemresponse','PD2T3 Modelresponse','Location','NorthEastOutside'); \n set(c,'Interpreter','none');" << std::endl;
    myfile << "title('" << file_name << " Velocity Stepresponse','interpreter','none')"  << std::endl;
    myfile << "grid" << std::endl;
    myfile << "subplot(2,1,2);" << std::endl;
    myfile << "plot(t," << file_name << "_dot_in,t," << file_name << "_dot_integrated,t,Gs_" << file_name << "_out_integrated3)" << std::endl;
    myfile << "c=legend('Velocity Input','Position Systemresponse','PID2T3 Modelresponse','Location','NorthEastOutside'); \n set(c,'Interpreter','none');" << std::endl;
    myfile << "title('" << file_name << " Position Stepresponse','interpreter','none')"  << std::endl;
    myfile << "grid" << std::endl;

    myfile.close();
}

