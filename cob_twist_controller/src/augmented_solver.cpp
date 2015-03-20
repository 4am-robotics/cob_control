#include "ros/ros.h"

#include "cob_twist_controller/augmented_solver.h"
#include <Eigen/Geometry>


augmented_solver::augmented_solver(const KDL::Chain& chain, double eps, int maxiter):
	chain_(chain),
	jac_(chain_.getNrOfJoints()),
	jnt2jac_(chain_),
	maxiter_(maxiter)
{}

augmented_solver::~augmented_solver()
{}


int augmented_solver::CartToJnt(const KDL::JntArray& q_in, const KDL::JntArray& last_q_dot, KDL::Twist& v_in, KDL::JntArray& qdot_out, std::vector<float> limits_min, std::vector<float> limits_max, KDL::Frame &base_position, KDL::Frame &chain_base)
{
	///Let the ChainJntToJacSolver calculate the jacobian "jac_chain" for the current joint positions "q_in"
	KDL::Jacobian jac_chain(chain_.getNrOfJoints());
	Eigen::Matrix<double,6,3> jac_b;
	
	jnt2jac_.JntToJac(q_in, jac_chain);
	
	if(params_.base_active)
	{
		Eigen::Matrix<double, 3, 3> chain_base_rot, base_rot, tip_base_rot;
		Eigen::Vector3d w_chain_base;
		Eigen::Vector3d r_chain_base;
		Eigen::Vector3d tangential_vel;
		Eigen::MatrixXd W_base_ratio = Eigen::MatrixXd::Identity(jac_.columns(), jac_.columns());
		
		double base_ratio = params_.base_ratio;
		
		//Create standard platform jacobian
		jac_b.setZero();
		
		// Get current x and y position from EE and chain_base with respect to base_footprint
		x_ = base_position.p.x();
		y_ = base_position.p.y();
		z_ = base_position.p.z();
		Eigen::Vector3d r_base_link(x_,y_,z_);
		
		chain_base_rot << 	chain_base.M.data[0],chain_base.M.data[1],chain_base.M.data[2],
                        chain_base.M.data[3],chain_base.M.data[4],chain_base.M.data[5],
                        chain_base.M.data[6],chain_base.M.data[7],chain_base.M.data[8];
		
		// Transform from base_link to chain_base
		Eigen::Vector3d w_base_link(0,0,base_ratio);
		//Eigen::Vector3d w_base_link(0,0,1);
		w_chain_base = chain_base_rot*w_base_link;
		r_chain_base = chain_base_rot*r_base_link;
		
		//Calculate tangential velocity
		tangential_vel = w_chain_base.cross(r_chain_base);
		
		 //Vx-Base <==> q8 effects a change in the following chain_base Vx velocities
		jac_b(0,0) = base_ratio*chain_base_rot(0,0);
		jac_b(0,1) = base_ratio*chain_base_rot(0,1);
		jac_b(0,2) = tangential_vel(0);
		
		// Vy-Base <==> q9 effects a change in the following chain_base Vy velocities
		jac_b(1,0) = base_ratio*chain_base_rot(1,0);
		jac_b(1,1) = base_ratio*chain_base_rot(1,1);
		jac_b(1,2) = tangential_vel(1);
		
		// Vz-Base <==>  effects a change in the following chain_base Vz velocities
		jac_b(2,0) = base_ratio*chain_base_rot(2,0);
		jac_b(2,1) = base_ratio*chain_base_rot(2,1);
		jac_b(2,2) = tangential_vel(2);
		
		//Phi <==> Wz with respect to base_link
		jac_b(3,2) = w_chain_base(0);
		jac_b(4,2) = w_chain_base(1);
		jac_b(5,2) = w_chain_base(2);
		
		//combine chain Jacobian and platform Jacobian
		Eigen::Matrix<double, 6, Eigen::Dynamic> jac_full;
		jac_full.resize(6,chain_.getNrOfJoints() + jac_b.cols());
		jac_full << jac_chain.data,jac_b;
		
		//std::cout << "Combined jacobian:\n " << jac_full << "\n";
		//ROS_INFO_STREAM("JacBase: rows " <<jac_base_.rows()<<"; cols "<<jac_base_.cols());
		//ROS_INFO_STREAM("JacFull: rows " <<jac_full.rows()<<"; cols "<<jac_full.cols());
		
		//ROS_INFO_STREAM("JacANTE: rows " <<jac_.rows()<<"; cols "<<jac_.columns());
		jac_.resize(chain_.getNrOfJoints() + jac_b.cols());
		//ROS_INFO_STREAM("JacPOST: rows " <<jac_.rows()<<"; cols "<<jac_.columns());
		
		jac_.data << jac_full;
	}
	else
	{
		jac_.resize(chain_.getNrOfJoints());
		jac_.data << jac_chain.data;
	}
	
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac_.data,Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::VectorXd S = svd.singularValues();
	
	double damping_factor = 0.0;
	if (params_.damping_method == MANIPULABILITY)
	{
		Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac_.data * jac_.data.transpose();
		double d = prod.determinant();
		double w = std::sqrt(std::abs(d));
		damping_factor = (w<params_.wt) ? (params_.lambda0 * pow((1 - w/params_.wt),2)) : 0.0;
		//std::cout << "w" << w << " wt" <<wt << " Condition" << (bool)(w<wt) << "\n";
		//std::cout << "w:" << w << "\n";
	}
  
	else if (params_.damping_method == CONSTANT)
	{
		damping_factor = params_.damping_factor;
	}
	
	else if (params_.damping_method == TRUNCATION)
	{
		damping_factor = 0.0;
	}
	else
	{
		ROS_ERROR("DampingMethod %d not defined! Aborting!", params_.damping_method);
		return -1;
	}
	//std::cout << "Daming_factor:" << damping_factor << "\n";
	
	int task_size = 1;
	//Weighting matrix for endeffector Jacobian Je (jac_)
	//Eigen::MatrixXd We = Eigen::MatrixXd::Identity(jac_.rows(), jac_.rows());
	
	//Weighting matrix for additional/task constraints Jc
	//Eigen::MatrixXd Wc = Eigen::MatrixXd::Identity(task_size,task_size);
	//Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(task_size,jac_.columns());
	
	//Weighting matrix for damping 
	Eigen::MatrixXd Wv = Eigen::MatrixXd::Identity(jac_.rows(), jac_.rows());
	
	Eigen::VectorXd v_in_vec = Eigen::VectorXd::Zero(jac_.rows());
	Eigen::VectorXd v_in_vec_base = Eigen::VectorXd::Zero(jac_base_.rows());
	Eigen::MatrixXd qdot_out_vec;
	Eigen::MatrixXd qdot_out_vec_enforced;
	
	///use calculated damping value lambda for SVD
	Wv = Wv*damping_factor*damping_factor; //why squared?
	
	///convert input
	for (int i=0; i<jac_.rows(); i++)
	{	v_in_vec(i)=v_in(i);	}
	
	///solution of the equation system	
	if(params_.JLA_active)
	{
		Eigen::MatrixXd W_jla = calculate_weighting(q_in, last_q_dot, limits_min, limits_max).asDiagonal();
		
		if(params_.damping_method == TRUNCATION)
		{
			Eigen::JacobiSVD<Eigen::MatrixXd> svdWholeMatrix(jac_.data*W_jla.inverse()*jac_.data.transpose()+Wv,Eigen::ComputeFullU | Eigen::ComputeFullV);
			Eigen::VectorXd SWholeMatrix = svdWholeMatrix.singularValues();
			Eigen::VectorXd S_inv = Eigen::VectorXd::Zero(SWholeMatrix.rows());
			
			for(int i=0;i<SWholeMatrix.rows();i++)
			{	S_inv(i) = (SWholeMatrix(i)<params_.eps) ? 0 : 1/SWholeMatrix(i);	}
			
			Eigen::MatrixXd tmp = svdWholeMatrix.matrixV()*S_inv.asDiagonal()*svdWholeMatrix.matrixV().transpose();
			qdot_out_vec = W_jla.inverse()*jac_.data.transpose()*tmp*v_in_vec;
		}
		
		else
		{
			if (jac_.columns()>=jac_.rows())
			{
				Eigen::MatrixXd tmp = (jac_.data*W_jla.inverse()*jac_.data.transpose()+Wv).inverse();
				qdot_out_vec = W_jla.inverse()*jac_.data.transpose()*tmp*v_in_vec;
			}
			else   //special case, the last formula is valid only for a full-row Jacobian
			{
				//non-redundant chain -> no damping Wv
				Eigen::MatrixXd W_specialcase = Eigen::MatrixXd::Identity(jac_.columns(), jac_.columns());
				for(int i=0;i<jac_.columns();i++)
				{
					W_specialcase(i,i)=sqrt(W_jla(i,i));
				}
				Eigen::MatrixXd tmp = (W_specialcase.inverse()*jac_.data.transpose()*jac_.data*W_specialcase.inverse()).inverse();
				qdot_out_vec = W_specialcase.inverse()*tmp*W_specialcase.inverse()*jac_.data.transpose()*v_in_vec;
			}
		}
	}
	else //JLA is not active
	{
		if(params_.damping_method == TRUNCATION)
		{
			Eigen::MatrixXd jac_pinv = Eigen::MatrixXd::Zero(jac_.columns(),jac_.rows());
			Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(jac_.columns(),jac_.rows());
			for (int i=0; i<S.rows(); i++)
			{
				for (int j=0; j<jac_.rows(); j++)
				{
					for (int k=0; k<jac_.columns(); k++)
					{
						double denominator = pow(S(i),2)+pow(damping_factor,2);
						double factor = (denominator < params_.eps) ? 0.0 : S(i)/denominator;
						jac_pinv(k,j)+=factor*svd.matrixV()(k,i)*svd.matrixU()(j,i);
					}
				}
			}
			qdot_out_vec = jac_pinv*v_in_vec;
		}
		else
		{
			if(jac_.columns()>=jac_.rows())
			{
				Eigen::MatrixXd tmp = (jac_.data*jac_.data.transpose()+Wv).inverse();
				qdot_out_vec=jac_.data.transpose()*tmp*v_in_vec;
			}
			else  //special case, the last formula is valid only for a full-row Jacobian
			{
				Eigen::MatrixXd Wv_specialcase = Eigen::MatrixXd::Identity(jac_.columns(), jac_.columns())*damping_factor;
				Eigen::MatrixXd tmp = (jac_.data.transpose()*jac_.data+Wv_specialcase).inverse();
				qdot_out_vec = tmp*jac_.data.transpose()*v_in_vec;
			}
		}
	}
	
	if(params_.enforce_limits)
		{	qdot_out_vec_enforced = enforce_limits(q_in, qdot_out_vec, limits_min, limits_max);	}
	else
		{	qdot_out_vec_enforced = qdot_out_vec;	}
	
	
	///convert output
	for(int i=0; i<jac_.columns(); i++)
	{	qdot_out(i)=qdot_out_vec_enforced(i);	}
	
	return 1;
}

Eigen::VectorXd augmented_solver::calculate_weighting(const KDL::JntArray& q, const KDL::JntArray& last_q_dot, std::vector<float> limits_min, std::vector<float> limits_max)
{
	//This function calculates the weighting matrix used to penalize a joint when it is near and moving towards a limit
	//The last joint velocity is used to determine if it that happens or not
	
	Eigen::VectorXd output = Eigen::VectorXd::Zero(jac_.columns());
  
	for(int i=0; i<jac_.columns() ; i++)
	{
		if(i<chain_.getNrOfJoints())	//See Chan paper
		{
			double dh = M_PI/180*fabs(pow(limits_max[i]-limits_min[i],2)*(2*q(i)-limits_max[i]-limits_min[i])/(4*pow(limits_max[i]-q(i),2)*pow(q(i)-limits_min[i],2)));
			//std::cout<<"dh:"<<dh<<std::endl;
			
			if((last_q_dot(i)>0 && ((limits_max[i]-q(i)) < (q(i)-limits_min[i]))) || (last_q_dot(i)<0 && ((limits_max[i]-q(i)) > (q(i)-limits_min[i]))))
			{	output(i) = 1+dh;	}
			else
			{	output(i) = 1;	}
		}
		else
		{	output(i) = 1;	}
	}
	
	return output;
}

Eigen::VectorXd augmented_solver::enforce_limits(const KDL::JntArray& q, Eigen::MatrixXd qdot_out, std::vector<float> limits_min, std::vector<float> limits_max)
{
	//This function multiplies the velocities that result from the IK, in case they violate the specified tolerance
	//This factor uses the cosine function to provide a smooth transition from 1 to zero
	//The factor is not used if the output causes the joint to move far from its limits
	
	Eigen::VectorXd output = Eigen::VectorXd::Zero(jac_.columns());
	double tolerance = params_.tolerance/180*M_PI;
	double factor=0;
	bool tolerance_surpassed=false;
	
	for(int i=0; i<jac_.columns() ;i++)
	{
		if(i<chain_.getNrOfJoints())
		{
			if(limits_max[i]-q(i)<tolerance)	//Joint is nearer to the maximum limit
			{
				if(qdot_out(i)>0)	//Joint moves towards the limit
				{
					double temp = 1/pow((0.5+0.5*cos(M_PI*(q(i)+tolerance-limits_max[i])/tolerance)),5);
					factor = (temp>factor) ? temp : factor;
					tolerance_surpassed = true;
				}
			}
			else
			{
				if(q(i)-limits_min[i]<tolerance)	//Joint is nearer to the minimum limit
				{
					if(qdot_out(i)<0)	//Joint moves towards the limit
					{
						double temp = 1/pow(0.5+0.5*cos(M_PI*(q(i)-tolerance-limits_min[i])/tolerance),5);
						factor = (temp>factor) ? temp : factor;
						tolerance_surpassed = true;
					}
				}
			}
		}
	}
	
	for(int i=0; i<jac_.columns() ;i++)
	{
		output(i) = tolerance_surpassed ? qdot_out(i)/factor : qdot_out(i);
	}
	
	return output;
}
