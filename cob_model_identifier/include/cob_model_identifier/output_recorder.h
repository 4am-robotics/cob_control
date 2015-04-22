
/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2014 \n
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
 *   ROS package name: cob_model_identifier
 *
 * \author
 *   Author: Christoph Mark, email: christoph.mark@ipa.fraunhofer.de
 *
 * \date Date of creation: September, 2014
 *
 * \brief
 *   ...
 *
 ****************************************************************/
#ifndef OUTPUT_RECORDER_H
#define OUTPUT_RECORDER_H

#include <termios.h>
#include <signal.h>
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/framevel.hpp>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <boost/thread.hpp>


class OutputRecorder
{
public:
	bool initialize();
	void run();
	void record(ros::Time);
	void getVelocity();
	void timerCallback(const ros::TimerEvent&);
	double calculateLS(std::vector<double>* vec_out, std::vector<double>* vec_in,int modellorder,double &a1,double &a2,double &a3,double &b1,double &b2, double &b3);
	void pseudoinverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &Minv, double tolerance);
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);
	void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void fillDataVectors(	double x_dot_lin_in, double x_dot_lin_out,
							double y_dot_lin_in, double y_dot_lin_out,
							double z_dot_lin_in, double z_dot_lin_out,
							double x_dot_rot_in, double x_dot_rot_out,
							double y_dot_rot_in, double y_dot_rot_out,
							double z_dot_rot_in, double z_dot_rot_out,
							double x_lin_out, double y_lin_out, double z_lin_out,double x_rot_out,double y_rot_out,double z_rot_out);
						 
	double calculateRLS(double processInput, double processOutput,int modellorder);
	void printModel(double a1, double b1, std::string axis);
	geometry_msgs::Pose getEndeffectorPose();
	geometry_msgs::Pose getTrackingFramePosition();

	void euler(std::vector<double> *out, double in, double dt);	
	void writeToMFile(std::string fileName,std::vector<double> *dot_in,std::vector<double> *dot_out,std::vector<double> *pos_out,std::vector<double> *dot_integrated);
	void stepResponsePlot(std::string fileName,std::vector<double> *in, std::vector<double> *x_lin_out,std::vector<double> *y_lin_out,std::vector<double> *z_lin_out,std::vector<double> *x_rot_out,std::vector<double> *y_rot_out,std::vector<double> *z_rot_out);
	
	void stop_recording();
	void quit(int sig);

	
	
private:
	ros::NodeHandle nh_;
	ros::Subscriber twist_sub_,jointstate_sub_;
	
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
	KDL::Vector vector_vel_,vector_rot_;
	
	/// Outputs
	// Velocity
	std::vector<double> x_dot_lin_vec_out_	,y_dot_lin_vec_out_	,z_dot_lin_vec_out_;
	std::vector<double> x_dot_rot_vec_out_	,y_dot_rot_vec_out_	,z_dot_rot_vec_out_;
		
	// Position
	std::vector<double> x_lin_vec_out_		,y_lin_vec_out_		,z_lin_vec_out_;
	std::vector<double> x_rot_vec_out_		,y_rot_vec_out_		,z_rot_vec_out_;
	
	
	double q_x_lin_out	,q_y_lin_out	,q_z_lin_out;
	
	/// Inputs
	std::vector<double> x_dot_lin_vec_in_	,y_dot_lin_vec_in_	,z_dot_lin_vec_in_;
	std::vector<double> x_dot_rot_vec_in_	,y_dot_rot_vec_in_	,z_dot_rot_vec_in_;
	
	double x_dot_lin_in_,y_dot_lin_in_,z_dot_lin_in_;
	double x_dot_rot_in_,y_dot_rot_in_,z_dot_rot_in_;
	double q_x_lin_in,q_y_lin_in,q_z_lin_in;
	
	bool finished_recording_;
	
	/// Transform Listener
	tf::TransformListener listener_;
	
	/// Euler Integration
	double dt_;
	
	bool start_;
	std::vector <double> timeVec;
	
	/// For Keyboard commands
	char c;
	int kfd;
	struct termios cooked, raw;
};

#endif

