
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
#include <cob_srvs/SetString.h>


class OutputRecorder
{
public:
	void initialize();
	void run();
	void record(ros::Time);
	void getVelocity();
	void timerCallback(const ros::TimerEvent&);
	double calculateLS(std::vector<double>* vec_out, std::vector<double>* vec_in,int modellorder,double &a1,double &a2,double &a3,double &b1,double &b2, double &b3);
	void pseudoinverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &Minv, double tolerance);	
	void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg);
	void twist_cb(const geometry_msgs::Twist::ConstPtr& msg);
	void fillDataVectors(double x_dot_lin_in, double x_dot_lin_out, double y_dot_lin_in, double y_dot_lin_out, double z_dot_lin_in, double z_dot_lin_out,
						 double x_lin_in_normalized, double y_lin_in_normalized, double z_lin_in_normalized,
						 double x_dot_rot_in, double x_dot_rot_out, double y_dot_rot_in, double y_dot_rot_out, double z_dot_rot_in, double z_dot_rot_out,
						 double x_rot_in_normalized, double y_rot_in_normalized, double z_rot_in_normalized,
						 double x_lin_out, double y_lin_out, double z_lin_out,double x_rot_out,double y_rot_out,double z_rot_out);
						 
	double calculateRLS(double processInput, double processOutput,int modellorder);
	void printModel(double a1, double b1, std::string axis);
	geometry_msgs::Pose getEndeffectorPose();
	geometry_msgs::Pose getTrackingFramePosition();
	void move_ptp(geometry_msgs::Pose targetPose);
	void start_tracking();
	void stop_tracking();
	void normalized_twist_cb(const geometry_msgs::Twist::ConstPtr& msg);
	void euler(std::vector<double> *out, double in, double dt);	
	void writeToMFile(std::string fileName,std::vector<double> *dot_in,std::vector<double> *dot_out,std::vector<double> *pos_out,std::vector<double> *dot_integrated, std::vector<double> *dot_normalized_in);
	Eigen::VectorXd getTheta(Eigen::MatrixXd &F, Eigen::VectorXd &y);
	void stepResponsePlot(std::string fileName,std::vector<double> *in, std::vector<double> *x_lin_out,std::vector<double> *y_lin_out,std::vector<double> *z_lin_out,std::vector<double> *x_rot_out,std::vector<double> *y_rot_out,std::vector<double> *z_rot_out);

	
private:
	ros::NodeHandle nh_;
	ros::Subscriber twist_sub_,twist_sub_norm_;
	ros::Subscriber jointstate_sub_;
	ros::Publisher twist_pub_,model_pub_;
	
	/// KDL Conversion
	KDL::Chain chain_;
	std::string chain_base_;
	std::string chain_tip_;
	KDL::JntArray last_q_;
	KDL::JntArray last_q_dot_;
	std::vector<std::string> joints_;
	KDL::JntArrayVel JntArrayVel_;
	KDL::ChainFkSolverVel_recursive* jntToCartSolver_vel_;
	unsigned int dof_;
	KDL::Vector vector_vel_,vector_rot_;
	double samples_;

	
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
	double q_x_lin_in	,q_y_lin_in		,q_z_lin_in;
	
	/// Normalized Inputs
	std::vector<double> trans_x_vect_in_normalized_,trans_y_vect_in_normalized_,trans_z_vect_in_normalized_;
	std::vector<double> rot_x_vect_in_normalized_,rot_y_vect_in_normalized_,rot_z_vect_in_normalized_;
	double x_lin_in_normalized_,y_lin_in_normalized_,z_lin_in_normalized_;
	double x_rot_in_normalized_,y_rot_in_normalized_,z_rot_in_normalized_;
	
	/// Frametracker
	ros::ServiceClient startTracking_;
	ros::ServiceClient stopTracking_;
	
	
	
	/// Transform Listener
   	tf::Transform transform_;
   	tf::Quaternion q_;
   	tf::TransformListener listener_;
   	tf::TransformBroadcaster br_;
   	tf::StampedTransform stampedTransform_;
	double roll_,pitch_,yaw_;
	
	std::vector <double> trans_x_,trans_x_punkt_,timeVect_;
	std::string referenceFrame_,endeffectorFrame_,trackingFrame_;
	ros::Timer timer_stop_;
	
	/// Euler Integration
	double dt_;
	
	std::vector <double> timeVec;
};

#endif

