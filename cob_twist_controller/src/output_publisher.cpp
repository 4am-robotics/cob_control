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
#include <cob_twist_controller/output_publisher.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;


void OutputPublisher::initialize()
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
	
	jointstate_sub_ = nh_.subscribe("/joint_states", 1, &OutputPublisher::jointstate_cb, this);
	end_eff_vel_pub_ = nh_.advertise<geometry_msgs::Twist> ("end_effector_vel", 1);
	end_eff_pos_pub_ = nh_.advertise<geometry_msgs::Twist> ("end_effector_pos", 1);
	manipulability_pub_ = nh_.advertise<std_msgs::Float64> ("manipulability", 1);
	rcond_pub_ = nh_.advertise<std_msgs::Float64> ("rcond", 1);
	
	ROS_INFO("...initialized!");
}



void OutputPublisher::run()
{	
	
	tf::Transform trans;	
	double x_lin_start	,y_lin_start	,z_lin_start;
	double x_rot_start	,y_rot_start	,z_rot_start;
	
	ROS_INFO("...running");
	
	geometry_msgs::Twist end_effector_vel_msg;
	geometry_msgs::Twist end_effector_pos_msg;
	std::vector <double> x_dot_lin_integrated,y_dot_lin_integrated,z_dot_lin_integrated, x_dot_rot_integrated,y_dot_rot_integrated,z_dot_rot_integrated;
	int iterations=0,samples=0;
	ros::Rate r(68.0);
	geometry_msgs::Pose q_soll,q_ist;
	
	// Transform RPY to Quaternion
	q_.setRPY(0,0,M_PI);
	trans.setRotation(q_);	
	
	ros::Time time = ros::Time::now();
	ros::Time last_update_time = time;
	ros::Duration period = time - last_update_time;	
	
	q_ist = getEndeffectorPose();
	x_lin_start = q_ist.position.x;
	y_lin_start = q_ist.position.y;
	z_lin_start = q_ist.position.z;
	
	initial_iteration=true;
	
	while (ros::ok()){	
		
		//ROS_WARN("In cycle");
		
		time = ros::Time::now();
		period = time - last_update_time;		
		
		q_ist = getEndeffectorPose();

		/// Ist Position
		q_x_lin_out = q_ist.position.x;
		q_y_lin_out = q_ist.position.y;
		q_z_lin_out = q_ist.position.z;
		
		//q_soll = getTrackingFramePosition();
		//q_x_lin_in = q_soll.position.x;
		//q_y_lin_in = q_soll.position.y;
		//q_z_lin_in = q_soll.position.z;
				
		euler(&x_dot_lin_integrated,vector_vel_.x(),period.toSec());
		euler(&y_dot_lin_integrated,vector_vel_.y(),period.toSec());
		euler(&z_dot_lin_integrated,vector_vel_.z(),period.toSec());
		euler(&x_dot_rot_integrated,vector_rot_.x(),period.toSec());
		euler(&y_dot_rot_integrated,vector_rot_.y(),period.toSec());
		euler(&z_dot_rot_integrated,vector_rot_.z(),period.toSec());
		
		end_effector_vel_msg.linear.x = vector_vel_.x();
		end_effector_vel_msg.linear.y = vector_vel_.y();
		end_effector_vel_msg.linear.z = vector_vel_.z();
		end_effector_vel_msg.angular.x = vector_rot_.x();
		end_effector_vel_msg.angular.y = vector_rot_.y();
		end_effector_vel_msg.angular.z = vector_rot_.z();
			
		end_eff_vel_pub_.publish(end_effector_vel_msg);
		
		//end_effector_pos_msg.linear.x = x_dot_lin_integrated.back();
		//end_effector_pos_msg.linear.y = y_dot_lin_integrated.back();
		//end_effector_pos_msg.linear.z = z_dot_lin_integrated.back();
		
		end_effector_pos_msg.linear.x = vector_pos_lin_.x();
		end_effector_pos_msg.linear.y = vector_pos_lin_.y();
		end_effector_pos_msg.linear.z = vector_pos_lin_.z();
		
		end_effector_pos_msg.angular.x = x_dot_rot_integrated.back();
		end_effector_pos_msg.angular.y = y_dot_rot_integrated.back();
		end_effector_pos_msg.angular.z = z_dot_rot_integrated.back();
		
		end_eff_pos_pub_.publish(end_effector_pos_msg);
		
		if(iterations > 2){
			dt_ += period.toSec();
			samples++;
		}
		
		iterations++;
		
		//if(samples >= 500){
			//dt_ /=  samples;
			//std::cout << "dt_ = " << dt_ <<std::endl;
			//break;
		//}
		
		
		last_update_time = time;
		ros::spinOnce();
		r.sleep();
		
		
	}
	

}


void OutputPublisher::jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg)
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
			
			
			KDL::Frame FramePos;
			jntToCartSolver_pos_ = new KDL::ChainFkSolverPos_recursive(chain_);
			KDL::JntArray jntArray = KDL::JntArray(last_q_);
			ret = jntToCartSolver_pos_->JntToCart(jntArray,FramePos,-1);
			
			if(ret>=0){			
				vector_pos_lin_= FramePos.p;
			}
			else{
				ROS_WARN("ChainFkSolverVel failed!");
			}	
			
			
		}
		
		KDL::ChainJntToJacSolver jnt2jac(chain_);
		KDL::Jacobian jac(chain_.getNrOfJoints());
		jnt2jac.JntToJac(last_q_,jac);
		
		//compute manipulability
        ///kappa = sqrt(norm(J*Jt))
        ///see  T.Yoshikawa "Manipulability of robotic mechanisms"
        ///     International Journal of Robotics Research, 4(2):3-9, 1985
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac.data * jac.data.transpose();
        double d = prod.determinant();
        double kappa = std::sqrt(std::abs(d));
        std_msgs::Float64 manipulability_msg;
        manipulability_msg.data=kappa;
        //
        //prod = jac.data.transpose() *jac.data;
        //std::cout<<"prod.norm() "<<prod.norm()<<std::endl;
        //std::cout<<"prod.inverse().norm() "<<prod.inverse().norm()<<std::endl;        
        //double rcond = 1/(prod.norm()*prod.inverse().norm());
        //
        //std_msgs::Float64 rcond_msg;
        //rcond_msg.data=rcond;
        
        //if (initial_iteration)
        //{
            //wkm1=kappa;
            //initial_iteration = false;
        //}
        //
        //manipulability_msg.data=(wkm1==0?0:kappa/wkm1);
        //
        //wkm1=kappa;
        
        manipulability_pub_.publish(manipulability_msg);
        //rcond_pub_.publish(rcond_msg);        
        
}

void OutputPublisher::euler(std::vector<double> *out, double in, double dt){
		if(out->size()==0){
			out->push_back(in*dt);
		}else{
			out->push_back(out->at(out->size()-1) + in*dt);
		}
}

geometry_msgs::Pose OutputPublisher::getEndeffectorPose()
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
