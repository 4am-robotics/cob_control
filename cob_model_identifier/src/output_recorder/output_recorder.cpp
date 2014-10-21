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
#include <ros/ros.h>
#include <tinyxml.h>
#include <vector>
#include <output_recorder/output_recorder.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;


void OutputRecorder::initialize()
{	
		///get params
	XmlRpc::XmlRpcValue jn_param;
	if (nh_.hasParam("joint_names"))
	{	
		nh_.getParam("joint_names", jn_param);	
	}
	else
	{	ROS_ERROR("Parameter joint_names not set");	}
	
	dof_ = jn_param.size();
	for(unsigned int i=0; i<dof_; i++)
	{	
		joints_.push_back((std::string)jn_param[i]);	
	}
	
	
	if (nh_.hasParam("base_link"))
	{
		nh_.getParam("base_link", chain_base_);
	}else{
			ROS_ERROR("no base link");
	}
	if (nh_.hasParam("tip_link"))
	{
		nh_.getParam("tip_link", chain_tip_);
	}
	if (nh_.hasParam("reference_frame"))
	{
		nh_.getParam("reference_frame", referenceFrame_);
	}
	
	if (nh_.hasParam("endeffector_frame"))
	{
		nh_.getParam("endeffector_frame", endeffectorFrame_);
	}
	
	if (nh_.hasParam("tracking_frame"))
	{
		nh_.getParam("tracking_frame", trackingFrame_);
	}
	
	///parse robot_description and generate KDL chains
	KDL::Tree my_tree;
	std::string robot_desc_string;
	nh_.param("/robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return;
	}
	my_tree.getChain(chain_base_, chain_tip_, chain_);
	if(chain_.getNrOfJoints() == 0)
	{
		ROS_ERROR("Failed to initialize kinematic chain");
		return;
	}
	
	///initialize variables and current joint values and velocities
	last_q_ = KDL::JntArray(chain_.getNrOfJoints());
	last_q_dot_ = KDL::JntArray(chain_.getNrOfJoints());
	
	jointstate_sub_ = nh_.subscribe("/joint_states", 1, &OutputRecorder::jointstate_cb, this);
	
	twist_sub_ = nh_.subscribe("/arm_controller/command_twist", 1, &OutputRecorder::twist_cb, this);
	twist_sub_norm_ = nh_.subscribe("/arm_controller/command_twist_normalized", 1, &OutputRecorder::normalized_twist_cb, this);
	twist_pub_ = nh_.advertise<geometry_msgs::Twist> ("command_twist", 1);
	model_pub_ = nh_.advertise<geometry_msgs::Twist> ("model_twist", 1);
	startTracking_ = nh_.serviceClient<cob_srvs::SetString>("/arm_controller/start_tracking");
	stopTracking_ = nh_.serviceClient<std_srvs::Empty>("/arm_controller/stop_tracking");

	
	ROS_INFO("...initialized!");
}



void OutputRecorder::run()
{	
	
	tf::Transform trans;
	double x_dot_lin_in	,y_dot_lin_in	,z_dot_lin_in;
	double x_dot_rot_in	,y_dot_rot_in	,z_dot_rot_in;
	double x_dot_lin_out,y_dot_lin_out	,z_dot_lin_out;
	double x_dot_rot_out,y_dot_rot_out	,z_dot_rot_out;
	double x_lin_start	,y_lin_start	,z_lin_start;
	double x_rot_start	,y_rot_start	,z_rot_start;	
	
	double x_lin_in_normalized,y_lin_in_normalized,z_lin_in_normalized;
	geometry_msgs::Twist twist_msg,model_twist_msg;
	std::vector <double> model,x_dot_lin_integrated,y_dot_lin_integrated,z_dot_lin_integrated;
	int iterations=0,samples=0;
	ros::Rate r(68.0);
	geometry_msgs::Pose q_soll,q_ist;
	
	geometry_msgs:: Pose ptpPose;
	
	// Transform RPY to Quaternion
	q_.setRPY(0,0,M_PI);
	trans.setRotation(q_);
	
	ptpPose.position.x = 0.4;
	ptpPose.position.y = 0.3;
	ptpPose.position.z = 0.5;
	ptpPose.orientation.x = trans.getRotation()[0];
	ptpPose.orientation.y = trans.getRotation()[1];
	ptpPose.orientation.z = trans.getRotation()[2];
	ptpPose.orientation.w = trans.getRotation()[3];

	ros::Time time = ros::Time::now();
	ros::Time last_update_time = time;
	ros::Duration period = time - last_update_time;
	start_tracking();
	
	q_ist = getEndeffectorPose();
	x_lin_start = q_ist.position.x;
	y_lin_start = q_ist.position.y;
	z_lin_start = q_ist.position.z;
	
	while (ros::ok()){	
		
		time = ros::Time::now();
		period = time - last_update_time;
		//move_ptp(ptpPose);
		
		q_ist = getEndeffectorPose();

		/// Ist Position
		q_x_lin_out = q_ist.position.x;
		q_y_lin_out = q_ist.position.y;
		q_z_lin_out = q_ist.position.z;
		

		/// Sollgeschwindigkeit
		x_dot_lin_in=x_dot_lin_in_;		
		y_dot_lin_in=y_dot_lin_in_;
		z_dot_lin_in=z_dot_lin_in_;
		
		x_dot_rot_in=x_dot_rot_in_;
		y_dot_rot_in=y_dot_rot_in_;
		z_dot_rot_in=z_dot_rot_in_;
		
		

		fillDataVectors(x_dot_lin_in ,vector_vel_.x() ,y_dot_lin_in ,vector_vel_.y() ,z_dot_lin_in ,vector_vel_.z(),x_lin_in_normalized_,y_lin_in_normalized_,z_lin_in_normalized_,
						x_dot_rot_in ,vector_rot_.x() ,y_dot_rot_in ,vector_rot_.y() ,z_dot_rot_in ,vector_rot_.z(),x_rot_in_normalized_,y_rot_in_normalized_,z_rot_in_normalized_,
						(q_x_lin_out+x_lin_start),(q_y_lin_out+y_lin_start),(q_z_lin_out+z_lin_start),0,0,0);
						

		
		euler(&x_dot_lin_integrated,vector_vel_.x(),period.toSec());
		euler(&y_dot_lin_integrated,vector_vel_.y(),period.toSec());
		euler(&z_dot_lin_integrated,vector_vel_.z(),period.toSec());
		
		if(iterations > 2){
			dt_ += period.toSec();	
			samples++;
		}
		
		iterations++;
		
		
		if(x_dot_lin_vec_in_.size() >= 500){
			dt_ /=  samples;
			std::cout << "dt_ = " << dt_ <<std::endl;
			break;
		}		
		last_update_time = time;
		ros::spinOnce();
		r.sleep();
	}
	
	
	/// Generate Octave Files
	writeToMFile("x_linear",&x_dot_lin_vec_in_,&x_dot_lin_vec_out_,&x_lin_vec_out_,&x_dot_lin_integrated);
	writeToMFile("y_linear",&y_dot_lin_vec_in_,&y_dot_lin_vec_out_,&y_lin_vec_out_,&y_dot_lin_integrated);
	writeToMFile("z_linear",&z_dot_lin_vec_in_,&z_dot_lin_vec_out_,&z_lin_vec_out_,&z_dot_lin_integrated);
}


void OutputRecorder::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
		KDL::JntArray q_temp = last_q_;
		KDL::JntArray q_dot_temp = last_q_dot_;
		int count = 0;
		
		for(unsigned int j = 0; j < dof_; j++)
		{
			for(unsigned int i = 0; i < msg->name.size(); i++)
			{
				if(strcmp(msg->name[i].c_str(), joints_[j].c_str()) == 0)
				{
					q_temp(j) = msg->position[i];
					q_dot_temp(j) = msg->velocity[i];
					count++;
					break;
				}
			}
		}
		
		if(count == joints_.size())
		{
			KDL::FrameVel FrameVel;
			last_q_ = q_temp;
			last_q_dot_ = q_dot_temp;
			KDL::JntArrayVel jntArrayVel = KDL::JntArrayVel(last_q_,last_q_dot_);

			jntToCartSolver_vel_ = new KDL::ChainFkSolverVel_recursive(chain_);
			int ret = jntToCartSolver_vel_->JntToCart(jntArrayVel,FrameVel,-1);
		
			if(ret>=0){			
				KDL::Twist twist = FrameVel.GetTwist();
				vector_vel_ = twist.vel;	
				vector_rot_ = twist.rot;
			}
			else{
				ROS_WARN("ChainFkSolverVel failed!");
			}	
		}
}



double OutputRecorder::calculateLS(std::vector<double>* vec_out, std::vector<double>* vec_in,int modellorder,double &a1,double &a2,double &a3,double &b1,double &b2, double &b3){
	double err=0;
	double err_old;
	int k=0;
	Eigen::MatrixXd F(vec_out->size()-modellorder,modellorder*2);	
	Eigen::VectorXd y(vec_out->size()-modellorder);
	Eigen::MatrixXd F_pinv;
	Eigen::MatrixXd theta;
	std::vector<double> errorVector;
	int d = 0; // delay
	
	while(true){	// While loop for defining delay time
		/// System 1. Ordnung
		if(modellorder==1){
			for(int i=0;i<vec_out->size()-modellorder;i++){
				if(i<d){
					F(i,0)=0;
					y(i) = 0;
				}else{
					F(i,0) = -1 * vec_out->at(i);	
					y(i) = vec_out->at(i+1);
				}	
				F(i,1) = vec_in->at(i);		

			}
		}
		

		/// System 2. Ordnung
		if(modellorder==2){
			for(int i=0;i<vec_out->size()-modellorder;i++){
				if(i<d){
					F(i,0) = 0;	
					F(i,1) = 0;	
					y(i) = 0;
				}else{
					F(i,0) = -1 * vec_out->at(i+1);	
					F(i,1) = -1 * vec_out->at(i);
					y(i) = vec_out->at(i+2);
				}
		
				F(i,2) = vec_in->at(i+1);	
				F(i,3) = vec_in->at(i);	
			}
		}

		/// System 3. Ordnung
		if(modellorder==3){
			for(int i=0;i<vec_out->size()-modellorder;i++){
				if(i<d){
					F(i,0) = 0;	
					F(i,1) = 0;	
					F(i,2) = 0;
					y(i) = 0;
				}else{
					F(i,0) = -1 * vec_out->at(i+2);	
					F(i,1) = -1 * vec_out->at(i+1);	
					F(i,2) = -1 * vec_out->at(i);
					y(i) = vec_out->at(i+3);
				}
				
				F(i,3) = vec_in->at(i+2);	
				F(i,4) = vec_in->at(i+1);	
				F(i,5) = vec_in->at(i);	
			}
		}
		
		

		
		pseudoinverse(F,F_pinv,(2.2204*pow(10,-16)));
		//Eigen::VectorXd theta = getTheta(F,y);
		theta = F_pinv * y;
		Eigen::MatrixXd e = y - F * theta;
		Eigen::MatrixXd e_squared = e.transpose() * e;
		

		err = e_squared(0,0);
		
		errorVector.push_back(err);
		d++;
		if(errorVector.size() > 1){
			if(errorVector.at(errorVector.size()-2) < errorVector.at(errorVector.size()-1)){
					d=errorVector.size()-1;
					break;
			}
		}	
	}
	
		a1=a2=a3=b1=b2=b3=0;
		if(modellorder==1){
			a1 = theta(0,0);
			b1 = theta(1,0);
		}
		
		if(modellorder==2){
			a1 = theta(0,0);
			a2 = theta(1,0);
			b1 = theta(2,0);
			b2 = theta(3,0);
		}
		
		if(modellorder==3){
			a1 = theta(0,0);
			a2 = theta(1,0);
			a3 = theta(2,0);
			b1 = theta(3,0);
			b2 = theta(4,0);
			b3 = theta(5,0);				
		}
	
		if(modellorder==4){
			a1 = theta(0,0);
			a2 = 1;
			b1 = theta(3,0);
			b2 = theta(4,0);
				
		}
	
	std::cout << "\n\nTheta: \n" << theta << std::endl;
	std::cout << "\n Delay: " << d << std::endl;
	std::cout << "\n Squared Error: \n" << err << std::endl;

	return errorVector.at(d);
}


/// Get theta, octave style.
Eigen::VectorXd OutputRecorder::getTheta(Eigen::MatrixXd &F, Eigen::VectorXd &y){
	int r=0;		
	Eigen::MatrixXd Q,R0,R1,R2;

	Eigen::MatrixXd A(F.rows(),F.cols()+1);
	A << F,y;


	Eigen::HouseholderQR<MatrixXd> qr(A);
	Q = qr.householderQ();
	R0 = Q.transpose()*A;

	
	for(int i =0;i<R0.rows();i++){
		for(int j=0;j<R0.cols();j++){
			if(fabs(R0(i,j)) < 2.2204*pow(10,-16)){
				R0(i,j)=0;
			}
		}
	}

	R1 = R0.block(0,0,R0.rows()-1,R0.cols()-1);
	R2 = R0.block(0,R0.cols()-1,R0.rows()-1,1);

	Eigen::JacobiSVD<Eigen::MatrixXd> svdOfF(R1, Eigen::ComputeThinU  | Eigen::ComputeThinV);
	const Eigen::MatrixXd U = svdOfF.matrixU();
	const Eigen::MatrixXd V = svdOfF.matrixV();
	const Eigen::VectorXd S = svdOfF.singularValues();
	
	Eigen::MatrixXd U_,V_;
	Eigen::VectorXd S_;
	
	double maxsv = 0 ;
	for (std::size_t i = 0; i < S.size(); ++i){
		if (fabs(S(i)) > maxsv){
			maxsv = fabs(S(i));
		}
	}
		
	for(int i=0;i<S.size();i++){	
		if(S(i) > (2.2204*pow(10,-16) * maxsv)){
			r++;
		}
	}
	
	V_ = V.leftCols(r);
	U_ = U.leftCols(r);
	Eigen::VectorXd temp = U_.transpose()*R2;
	Eigen::VectorXd temp2 = temp.cwiseQuotient(S);
	
	return  V_*temp2;
}


void OutputRecorder::pseudoinverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &Minv, double tolerance)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svdOfM(M, Eigen::ComputeThinU  | Eigen::ComputeThinV);
	const Eigen::MatrixXd U = svdOfM.matrixU();
	const Eigen::MatrixXd V = svdOfM.matrixV();
	const Eigen::VectorXd S = svdOfM.singularValues();
	Eigen::VectorXd Sinv = S;

	double maxsv = 0 ;
	for (std::size_t i = 0; i < S.rows(); ++i){
		if (fabs(S(i)) > maxsv){
			maxsv = fabs(S(i));
		}
	}
	
	for (std::size_t i = 0; i < S.rows(); ++i)
	{
		Sinv(i)=((S(i)< 0.0001 )?0:1/S(i));
	}
	
	Minv = V * Sinv.asDiagonal() * U.transpose();
}


void OutputRecorder::twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
		x_dot_lin_in_ = msg->linear.x;
		y_dot_lin_in_ = msg->linear.y;
		z_dot_lin_in_ = msg->linear.z;
		
		x_dot_rot_in_ = msg->angular.x;
		y_dot_rot_in_ = msg->angular.y;
		z_dot_rot_in_ = msg->angular.z;	
		
}


void OutputRecorder::fillDataVectors(double x_dot_lin_in, double x_dot_lin_out, double y_dot_lin_in, double y_dot_lin_out, double z_dot_lin_in, double z_dot_lin_out,
									 double x_lin_in_normalized, double y_lin_in_normalized, double z_lin_in_normalized,
									 double x_dot_rot_in, double x_dot_rot_out, double y_dot_rot_in, double y_dot_rot_out, double z_dot_rot_in, double z_dot_rot_out,
									 double x_rot_in_normalized, double y_rot_in_normalized, double z_rot_in_normalized,
									 double x_lin_out, double y_lin_out, double z_lin_out,double x_rot_out,double y_rot_out,double z_rot_out){
	/// lin velocity
	x_dot_lin_vec_in_.push_back(x_dot_lin_in);
	x_dot_lin_vec_out_.push_back(x_dot_lin_out);
	y_dot_lin_vec_in_.push_back(y_dot_lin_in);
	y_dot_lin_vec_out_.push_back(y_dot_lin_out);
	z_dot_lin_vec_in_.push_back(z_dot_lin_in);
	z_dot_lin_vec_out_.push_back(z_dot_lin_out);
	
	trans_x_vect_in_normalized_.push_back(x_lin_in_normalized);
	trans_y_vect_in_normalized_.push_back(y_lin_in_normalized);
	trans_z_vect_in_normalized_.push_back(z_lin_in_normalized);
	
	/// rot velocity
	x_dot_rot_vec_in_.push_back(x_dot_rot_in);
	x_dot_rot_vec_out_.push_back(x_dot_rot_out);
	y_dot_rot_vec_in_.push_back(y_dot_rot_in);
	y_dot_rot_vec_out_.push_back(y_dot_rot_out);
	z_dot_rot_vec_in_.push_back(z_dot_rot_in);
	z_dot_rot_vec_out_.push_back(z_dot_rot_out);
	
	rot_x_vect_in_normalized_.push_back(x_rot_in_normalized);	
	rot_y_vect_in_normalized_.push_back(y_rot_in_normalized);
	rot_z_vect_in_normalized_.push_back(z_rot_in_normalized);
	
	/// lin position
	x_lin_vec_out_.push_back(x_lin_out);
	y_lin_vec_out_.push_back(y_lin_out);
	z_lin_vec_out_.push_back(z_lin_out);
	
	/// rot position
	x_rot_vec_out_.push_back(x_rot_out);
	y_rot_vec_out_.push_back(y_rot_out);
	z_rot_vec_out_.push_back(z_rot_out);
}


void OutputRecorder::printModel(double a1, double b1, std::string axis){

	if(a1<0){
		std::cout << "Gz_" << axis << "=" << b1 << "*z^-1/(1" << a1 << "*z^-1)" << std::endl;
	}else{
		std::cout << "Gz_" << axis << "=" << b1 << "*z^-1/(1+" << a1 << "*z^-1)" << std::endl;		
	}
	std::cout << "Gs_" << axis << "=d2c(Gz_" << axis << ",'zoh')" << std::endl;	
}

geometry_msgs::Pose OutputRecorder::getEndeffectorPose()
{	
	ros::Time now = ros::Time::now();
	geometry_msgs::Pose pos;	
	tf::StampedTransform stampedTransform;
	bool transformed=false;


	// Get transformation
	try{
	listener_.waitForTransform(referenceFrame_,endeffectorFrame_, now, ros::Duration(0.5));
	listener_.lookupTransform(referenceFrame_,endeffectorFrame_, now, stampedTransform);
	}
	catch (tf::TransformException &ex) {
	ROS_ERROR("%s",ex.what());
	}
	
	
	pos.position.x=stampedTransform.getOrigin().x();
	pos.position.y=stampedTransform.getOrigin().y();
	pos.position.z=stampedTransform.getOrigin().z();
	pos.orientation.x = stampedTransform.getRotation()[0];
	pos.orientation.y = stampedTransform.getRotation()[1];
	pos.orientation.z = stampedTransform.getRotation()[2];
	pos.orientation.w = stampedTransform.getRotation()[3];
			
	return pos;
}


geometry_msgs::Pose OutputRecorder::getTrackingFramePosition()
{	
	ros::Time now = ros::Time::now();
	geometry_msgs::Pose pos;	
	tf::StampedTransform stampedTransform;
	bool transformed=false;
	
	// Get transformation
	try{
	listener_.waitForTransform(referenceFrame_,trackingFrame_, now, ros::Duration(0.5));
	listener_.lookupTransform(referenceFrame_,trackingFrame_, now, stampedTransform);
	}
	catch (tf::TransformException &ex) {
	ROS_ERROR("%s",ex.what());
	}
	
	pos.position.x=stampedTransform.getOrigin().x();
	pos.position.y=stampedTransform.getOrigin().y();
	pos.position.z=stampedTransform.getOrigin().z();
	pos.orientation.x = stampedTransform.getRotation()[0];
	pos.orientation.y = stampedTransform.getRotation()[1];
	pos.orientation.z = stampedTransform.getRotation()[2];
	pos.orientation.w = stampedTransform.getRotation()[3];
			
	return pos;
}



// Pseudo PTP
void OutputRecorder::move_ptp(geometry_msgs::Pose targetPose){

	ros::Time now;
	tf::StampedTransform stampedTransform;
	tf::Quaternion q;


	now = ros::Time::now();

	// Linearkoordinaten
	transform_.setOrigin( tf::Vector3(targetPose.position.x,targetPose.position.y,targetPose.position.z) );

	q = tf::Quaternion(targetPose.orientation.x,targetPose.orientation.y,targetPose.orientation.z,targetPose.orientation.w);
	transform_.setRotation(q);

	// Send br Frame
	br_.sendTransform(tf::StampedTransform(transform_, now, referenceFrame_, trackingFrame_));
	
}




void OutputRecorder::start_tracking()
{	
    cob_srvs::SetString start;
    start.request.data = trackingFrame_;
    startTracking_.call(start);
    
    if(start.response.success==true){
		ROS_INFO("...service called!");
	}
	else{
		ROS_INFO("...service failed");
	}
}


void OutputRecorder::stop_tracking()
{
    std_srvs::Empty srv_save_stop;
    srv_save_stop.request;
    if (stopTracking_.call(srv_save_stop)) {
		ROS_INFO("... service stopped!");
		}
    	else {
			ROS_ERROR("... service stop failed! FATAL!");
		}
}

void OutputRecorder::normalized_twist_cb(const geometry_msgs::Twist::ConstPtr& msg){
		x_lin_in_normalized_ = msg->linear.x;
		y_lin_in_normalized_ = msg->linear.y;
		z_lin_in_normalized_ = msg->linear.z;	
		x_rot_in_normalized_ = msg->angular.x;
		y_rot_in_normalized_ = msg->angular.y;
		z_rot_in_normalized_ = msg->angular.z;
}

void OutputRecorder::euler(std::vector<double> *out, double in, double dt){
		if(out->size()==0){
			out->push_back(in*dt);
		}else{
			out->push_back(out->at(out->size()-1) + in*dt);
		}
}

void OutputRecorder::writeToMFile(std::string fileName,std::vector<double> *dot_in,std::vector<double> *dot_out,std::vector<double> *pos_out,std::vector<double> *dot_integrated){
	std::ofstream myfile;
	std::string name;
	std::vector <double> errVec;
	double a1,a2,a3,b1,b2,b3=0;
		std::ostringstream a1_str,a2_str,a3_str,b1_str,b2_str,b3_str;

	name = "/home/fxm-cm_local/m-files/" +fileName + ".m";
	const char* charPath = name.c_str();

	myfile.open (charPath);
	
	myfile << "clear all;close all;\n\n";
  
	myfile << fileName << "_dot_in = [" << std::endl;
	for (int i=0;i<dot_in->size()-1;i++){
		myfile << dot_in->at(i) <<std::endl;
	}
    myfile << "]; \n" << std::endl;
  
  
  
	myfile << fileName << "_dot_out = [" << std::endl;
	for (int i=0;i<dot_out->size()-1;i++){
		myfile << dot_out->at(i) <<std::endl;
	}
    myfile << "]; \n" << std::endl;
    
    
    
    
	myfile << fileName << "_pos_out = [" << std::endl;
	for (int i=0;i<pos_out->size()-1;i++){
		myfile << pos_out->at(i) <<std::endl;
	}
    myfile << "]; \n" << std::endl;
    
    
    
	myfile << fileName << "_dot_integrated = [" << std::endl;
	for (int i=0;i<dot_integrated->size()-1;i++){
		myfile << dot_integrated->at(i) <<std::endl;
	}
    myfile << "]; \n" << std::endl;
    
	//myfile << "t = linspace(0,size(" << fileName << "_dot_in*" << dt_ <<"),size(" <<fileName << "_dot_in));" << std::endl;
	myfile << "t = linspace(0,5,size(" <<fileName << "_dot_in));" << std::endl;

    /// Generate Velocity Model --------------------------------------------------------------------------------------------------------------------
    //myfile << "s = tf('s'); z=tf('z'," << dt_ << ");" << std::endl;
	myfile << "s = tf('s'); z=tf('z',1/68);" << std::endl;
	
	
	/// 1. Ordnung
	a1 = b1 = a2 = b2 = 0;
	errVec.push_back(calculateLS(dot_out,dot_in,1,a1,a2,a3,b1,b2,b3));
	
	
	if(a1<0){
		myfile << "Gz_" << fileName << "1=" << b1 << "*z^-1/(1" << a1 << "*z^-1);" << std::endl;
	}else{
		myfile << "Gz_" << fileName << "1=" << b1 << "*z^-1/(1+" << a1 << "*z^-1);" << std::endl;		
	}
	
	
	
	/// 2. Ordnung
	a1_str.str("");a2_str.str("");a3_str.str("");b1_str.str("");b2_str.str("");b3_str.str("");
	a1_str.clear();a2_str.clear();a3_str.clear();b1_str.clear();b2_str.clear();b3_str.clear();
	
	errVec.push_back(calculateLS(dot_out,dot_in,2,a1,a2,a3,b1,b2,b3));
	
	a1_str << a1;
	a2_str << a2;

	b1_str << b1;
	b2_str << b2;

	
	std::string Gz2 = "Gz_" + fileName + "2=(" + b1_str.str() + "*z^-1 +" + b2_str.str() + "*z^-2)/(1 +" + a1_str.str() + "*z^-1 +" + a2_str.str() + "*z^-2);";
	while (Gz2.find("+-") != std::string::npos)
        Gz2.replace(Gz2.find("+-"), 2, "-");
       
	myfile << Gz2;
	
	

	
	
	/// 3. Ordnung
	a1_str.str("");a2_str.str("");a3_str.str("");b1_str.str("");b2_str.str("");b3_str.str("");
	a1_str.clear();a2_str.clear();a3_str.clear();b1_str.clear();b2_str.clear();b3_str.clear();
	
	errVec.push_back(calculateLS(dot_out,dot_in,3,a1,a2,a3,b1,b2,b3));
	
	a1_str << a1;
	a2_str << a2;
	a3_str << a3;
	b1_str << b1;
	b2_str << b2;
	b3_str << b3;
	
	std::string Gz3 = "Gz_" + fileName + "3=(" + b1_str.str() + "*z^-1 +" + b2_str.str() + "*z^-2 +" + b3_str.str() + "*z^-3)/(1 +" + a1_str.str() + "*z^-1 +" + a2_str.str() + "*z^-2 +" + a3_str.str() + "*z^-3);";
	while (Gz3.find("+-") != std::string::npos)
        Gz3.replace(Gz3.find("+-"), 2, "-");
       
	myfile << Gz3;
	

	
	/// Errorplot to identify model order
	myfile << "e =[" << errVec.at(0) << "," << errVec.at(1) << ","  << errVec.at(2) << "];" << std::endl;
	myfile << "figure; semilogy(e)" << std::endl;
	
	
	double step;
	for(int i = 0;i<dot_in->size()-1;i++){
		step+=dot_in->at(i);
	}
	step/=(dot_in->size()-1);
	std::cout << "step = " << step <<std::endl;
	
	
	int length = (int)pos_out->size()*2/3 - (int)pos_out->size()/3;
	double Ki = (pos_out->at((int)pos_out->size()*2/3) - pos_out->at((int)pos_out->size()/3))/(length * dt_) / step;


	ROS_INFO("length: %i	dY: %f   Ki: %f",length,(pos_out->at((int)pos_out->size()*2/3) - pos_out->at((int)pos_out->size()/3)),Ki);


	myfile << "Gs_" << fileName << "1=d2c(Gz_" << fileName << "1,'zoh');" << std::endl;
	myfile << "Gs_" << fileName << "2=d2c(Gz_" << fileName << "2,'zoh');" << std::endl;
	myfile << "Gs_" << fileName << "3=d2c(Gz_" << fileName << "3,'zoh');" << std::endl;
	
	
	//myfile << "Gt = tf(1,1,'iodelay',0.1); Gs_" << fileName << "*=Gt;"  << std::endl;
	/// Integrate every Model
    myfile << "Gs_" << fileName << "_integrated1 = Gs_" << fileName << "1*(" << Ki << "/s);" << std::endl;
    myfile << "Gs_" << fileName << "_integrated2 = Gs_" << fileName << "2*(" << Ki << "/s);" << std::endl;
    myfile << "Gs_" << fileName << "_integrated3 = Gs_" << fileName << "3*(" << Ki << "/s);" << std::endl;
    
    /// Simulate the Models
	myfile << "Gs_" << fileName << "_out1 = lsim(Gs_" << fileName << "1," << fileName << "_dot_in,t);" << std::endl;
	myfile << "Gs_" << fileName << "_out2 = lsim(Gs_" << fileName << "2," << fileName << "_dot_in,t);" << std::endl;
	myfile << "Gs_" << fileName << "_out3 = lsim(Gs_" << fileName << "3," << fileName << "_dot_in,t);" << std::endl;
	
	/// Simulate the integrated Models
	myfile << "Gs_" << fileName << "_out_integrated1 = lsim(Gs_" << fileName << "_integrated1," << fileName << "_dot_in,t);" << std::endl;
	myfile << "Gs_" << fileName << "_out_integrated2 = lsim(Gs_" << fileName << "_integrated2," << fileName << "_dot_in,t);" << std::endl;
	myfile << "Gs_" << fileName << "_out_integrated3 = lsim(Gs_" << fileName << "_integrated3," << fileName << "_dot_in,t);" << std::endl;
	
	/// Plot the Models and the real System response
	myfile << "figure" << std::endl;
	myfile << "plot(t," << fileName << "_dot_in,t," << fileName << "_dot_out,t,Gs_" << fileName << "_out1,t,Gs_" << fileName << "_out2,t,Gs_" << fileName << "_out3)" << endl;  
    myfile << "c=legend('Input','Response','PT1','PT2','PT3'); \n set(c,'Interpreter','none');" << std::endl;
    myfile << "title('" << fileName << " step response','interpreter','none')"  << std::endl;
    myfile << "grid" << std::endl;
	
	/// Plot everything
	myfile << "figure" << std::endl;
	myfile << "plot(t," << fileName << "_dot_in,t," << fileName << "_dot_out,t," << fileName << "_pos_out,t," << fileName << "_dot_integrated,t,Gs_" << fileName << "_out_integrated2)" << std::endl;
    myfile << "c=legend('" << fileName << "-dot-in','" << fileName << "-dot-out','" << fileName << "-pos-out','" << fileName << "-dot-out-integrated','PT2 Velocitymodel','" << "IT1_Positionmodel'); \n set(c,'Interpreter','none');" << std::endl;
    myfile << "title('" << fileName << " step response','interpreter','none')"  << std::endl;
    myfile << "grid" << std::endl;
    
    /// Plot step response
    myfile << "t1 = linspace(0,size(" << fileName << "_dot_in),1/68);" << std::endl;
	myfile << "figure" << std::endl;
	myfile << "plot(t," << fileName << "_dot_in,t," << fileName << "_dot_out,t," << fileName << "_pos_out,t," << fileName << "_dot_integrated)" << std::endl;
    myfile << "c=legend('" << fileName << "_dot_in','" << fileName << "_dot_out','" << fileName << "_pos_out','"  << fileName << "_dot_integrated'); \n set(c,'Interpreter','none');" << std::endl;
    myfile << "title('" << fileName << " step response','interpreter','none')"  << std::endl;
    myfile << "grid" << std::endl;
    
  myfile.close();
} 


Eigen::VectorXd OutputRecorder::lsmr(MatrixXd & A, VectorXd & b)
{
	double a_tol = 1e-6; double b_tol = 1e-6;
	double lambda = 0; double conlim = 1e+8;
	VectorXd u = b;
	double beta = u.norm();
	u.normalize();
	VectorXd v = A.transpose()*u;
	double alpha = v.norm();
	v.normalize();
	int m = A.rows(); int n = A.cols();
	int minDim = min(m, n);
	int maxiters = minDim;
	// skip local ortho...
	// vars for first iter
	int iter = 0;
	double zetabar = beta*alpha;
	double alphabar = alpha;
	double rho = 1.0;
	double rhobar = 1.0;
	double cbar = 1.0;
	double sbar = 0.0;
	VectorXd h = v;
	VectorXd hbar = VectorXd::Zero(n);
	VectorXd x = VectorXd::Zero(n);
	// vars for estimating ||r||
	double betadd = beta;
	double betad = 0.0;
	double rhodold = 1.0;
	double tautildeold = 0.0;
	double thetatilde = 0.0;
	double zeta = 0.0;
	double d = 0.0;
	// init vars for estimating ||A|| and cond(A)
	double normA2 = alpha*alpha;
	double maxrbar = 0.0;
	double minrbar = 1e100;
	// stopping crit
	double normb = beta;
	//double istop = 0;
	double ctol = 0;
	if (conlim > 0)
	ctol = 1.0/conlim;
	double normr = beta;
	double normAr = alpha*beta;
	if (normAr == 0) {
	cout << "exact solution is 0" << endl;
	return x;
	}
	while (iter < maxiters) { // loop count = 100
	u = A*v - alpha*u;
	beta = u.norm();
	if (beta > 0) {
	u.normalize();
	v = A.transpose()*u - beta*v;
	alpha = v.norm();
	if (alpha > 0)
	v.normalize();
	}
	// construct rotation Qhat
	double alphahat = sqrt(alphabar*alphabar + lambda*lambda); // no regularization term
	double chat = alphabar/alphahat;
	double shat = lambda/alphahat;
	// plane rotations...
	double rhoold = rho;
	rho = sqrt(alphahat*alphahat + beta*beta);
	double c = alphahat/rho;
	double s = beta/rho;
	double thetanew = s*alpha;
	alphabar = c*alpha;
	double rhobarold = rhobar;
	double zetaold = zeta;
	double thetabar = sbar*rho;
	double rhotemp = cbar*rho;
	rhobar = sqrt( cbar*rho*cbar*rho + thetanew*thetanew );
	cbar *= rho/rhobar;
	sbar = thetanew/rhobar;
	zeta = cbar*zetabar;
	zetabar = -sbar*zetabar;
	// update h, h_hat, x
	//cout << thetabar << ", " << rho << ", " << rhoold << ", " << rhobarold << endl;
	hbar = h - (thetabar*rho/(rhoold*rhobarold))*hbar;
	//cout << hbar << endl;
	//cout << zeta << ", " << rho << ", " << rhobar << endl;
	x += (zeta/(rho*rhobar))*hbar;
	//cout << x << endl;
	h = v - (thetanew/rho)*h;
	// estimate of ||r||
	double betaacute = chat*betadd;
	double betacheck = -shat*betadd;
	double betahat = c*betaacute;
	betadd = -s*betaacute;
	double thetatildeold = thetatilde;
	double rhotildeold = sqrt( rhodold*rhodold + thetabar*thetabar );
	double ctildeold = rhodold/rhotildeold;
	double stildeold = thetabar/rhotildeold;
	thetatilde = stildeold*rhobar;
	rhodold = ctildeold*rhobar;
	betad = -stildeold*betad + ctildeold*betahat;
	tautildeold = (zetaold - thetatildeold*tautildeold)/rhotildeold;
	double taud = (zeta - thetatilde*tautildeold)/rhodold;
	d = d + betacheck*betacheck;
	normr = sqrt(d + (betad - taud)*(betad - taud) + betadd*betadd);
	// estimate ||A||
	normA2 += beta*beta;
	double normA = sqrt(normA2);
	normA2 += alpha*alpha;
	maxrbar = max(maxrbar, rhobarold);
	if (iter > 1)
	minrbar = min(minrbar, rhobarold);
	double condA = max(maxrbar, rhotemp)/min(minrbar, rhotemp);
	// stopping crtierion
	normAr = abs(zetabar);
	double normx = x.norm();
	double test1 = normr/normb;
	double test2 = normAr / (normA*normr);
	double test3 = 1.0/condA;
	double t1 = test1 / (1 + normA*normx/normb);
	double rtol = b_tol + a_tol*normA*normx/normb;
	// skip error checking
	// check tests
	if (test3 <= ctol || test2 <= a_tol || test1 <= rtol)
	break;
	//printf("%d\t%f\t%0.3f\t%0.3f\t%f\t%0.1f\n", iter, x(0), normr, normAr, test1,test2);
	iter++;
	}
	//cout << x << endl;
	return x;
}

