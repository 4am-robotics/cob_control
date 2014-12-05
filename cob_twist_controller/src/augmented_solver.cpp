#include "ros/ros.h"

#include "cob_twist_controller/augmented_solver.h"
#include <fstream>

#define DEBUG true


augmented_solver::augmented_solver(const KDL::Chain& _chain, double _eps, int _maxiter):
    chain(_chain),
    jac(chain.getNrOfJoints()),
    jnt2jac(chain),
    maxiter(_maxiter),
    initial_iteration(true)
{}

augmented_solver::~augmented_solver()
{}


int augmented_solver::CartToJnt(const KDL::JntArray& q_in, KDL::Twist& v_in, KDL::JntArray& qdot_out, KDL::Frame &base_position, KDL::Frame &chain_base)
{
    ///used only for debugging
    std::ofstream file("test_end_effect.txt", std::ofstream::app);
    
    ///Let the ChainJntToJacSolver calculate the jacobian "jac_chain" for the current joint positions "q_in"
    KDL::Jacobian jac_chain(chain.getNrOfJoints());
    Eigen::Matrix<double,6,3> jac_b;

    jnt2jac.JntToJac(q_in, jac_chain);
    
    if(params_.base_active)
    {
        //Create standard platform jacobian
        jac_b.setZero();
        
        // Get current x and y position from EE and chain_base with respect to base_footprint
        x_ = base_position.p.x();
        y_ = base_position.p.y();

		Eigen::Matrix<double, 3, 3> chain_base_rot,base_rot;
		
		chain_base_rot << 	chain_base.M.data[0],chain_base.M.data[1],chain_base.M.data[2],
							chain_base.M.data[3],chain_base.M.data[4],chain_base.M.data[5],
							chain_base.M.data[6],chain_base.M.data[7],chain_base.M.data[8];
							
		std::cout <<"chain_base_rot: \n" << chain_base_rot << "\n";
		
		
		// Transform Wz from base_link to chain_base
		Eigen::Vector3d w_chain_base;
		Eigen::Vector3d w_base(0,0,params_.base_ratio);
		w_chain_base = chain_base_rot * w_base;
		
		// Calculate tangential velocity
		Eigen::Vector3d x_tangential_vel,y_tangential_vel,z_tangential_vel;
		Eigen::Vector3d x_chain_base(chain_base.M.data[0],chain_base.M.data[3],chain_base.M.data[6]);
		Eigen::Vector3d y_chain_base(chain_base.M.data[1],chain_base.M.data[4],chain_base.M.data[7]);
		/// Special case, If a base rotation causes a z linear velocity when the torso is bent.
		Eigen::Vector3d z_chain_base(chain_base.M.data[2],chain_base.M.data[5],chain_base.M.data[8]);
		
		
		x_tangential_vel = x_chain_base.cross(w_chain_base);
		y_tangential_vel = y_chain_base.cross(w_chain_base);
		z_tangential_vel = z_chain_base.cross(w_chain_base);
		
		 //Vx-Base <==> q8 effects a change in the following chain_base Vx velocities
		jac_b(0,0) = params_.base_ratio*chain_base_rot(0,0);   
		jac_b(0,1) = params_.base_ratio*chain_base_rot(0,1);
		jac_b(0,2) = x_tangential_vel(0) + y_tangential_vel(0) + z_tangential_vel(0);

		// Vy-Base <==> q9 effects a change in the following chain_base Vy velocities
		jac_b(1,0) = params_.base_ratio*chain_base_rot(1,0);   
		jac_b(1,1) = params_.base_ratio*chain_base_rot(1,1);
		jac_b(1,2) = x_tangential_vel(1) + y_tangential_vel(1) + z_tangential_vel(1);
        
        // Vz-Base <==>  effects a change in the following chain_base Vz velocities
		jac_b(2,0) = params_.base_ratio*chain_base_rot(2,0);   
		jac_b(2,1) = params_.base_ratio*chain_base_rot(2,1);
		jac_b(2,2) = x_tangential_vel(2) + y_tangential_vel(2) + z_tangential_vel(2);
		
		//Phi <==> Wz with respect to base_link
		jac_b(3,2) = w_chain_base(0);
		jac_b(4,2) = w_chain_base(1);
		jac_b(5,2) = w_chain_base(2);

        //combine chain Jacobian and platform Jacobian
        Eigen::Matrix<double, 6, Eigen::Dynamic> jac_full;
        jac_full.resize(6,chain.getNrOfJoints() + jac_b.cols());
        jac_full << jac_chain.data,jac_b;
        
        std::cout << "Combined jacobian:\n " << jac_full << "\n";
        //ROS_INFO_STREAM("JacBase: rows " <<jac_base.rows()<<"; cols "<<jac_base.cols());
        //ROS_INFO_STREAM("JacFull: rows " <<jac_full.rows()<<"; cols "<<jac_full.cols());
        
        //ROS_INFO_STREAM("JacANTE: rows " <<jac.rows()<<"; cols "<<jac.columns());
        jac.resize(chain.getNrOfJoints() + jac_b.cols());
        //ROS_INFO_STREAM("JacPOST: rows " <<jac.rows()<<"; cols "<<jac.columns());
        
        jac.data << jac_full;
    }
    else
    {
        jac.resize(chain.getNrOfJoints());
        jac.data << jac_chain.data;
    }
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac.data,Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd S = svd.singularValues();
    
    double damping_factor = 0.0;
    if (params_.damping_method == MANIPULABILITY)
    {
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac.data * jac.data.transpose();
        double d = prod.determinant();
        double w = std::sqrt(std::abs(d));
        damping_factor = ((w<params_.wt) ? (params_.lambda0 * pow((1 - w/params_.wt),2)) : 0);
        //std::cout << "w" << w << " wt" <<wt << " Condition" << (bool)(w<wt) << "\n";
    }
    
    else if (params_.damping_method == MANIPULABILITY_RATE)
    {
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac.data * jac.data.transpose();
        double d = prod.determinant();
        double w = std::sqrt(std::abs(d));
        if (initial_iteration)
        {
            damping_factor = params_.lambda0;
            initial_iteration = false;
        }
        else
        {
            if(wkm1 == 0) //division by zero
            {    damping_factor = params_.lambda0;    }
            else
            {    damping_factor = ((std::fabs(w/wkm1) > params_.wt) ? (params_.lambda0 * (1-w/wkm1)) : 0);    }
        }
        //std::cout<<"w: "<<w<<" wk-1: "<< wkm1 << " condition:" << (bool)(w/wkm1 <params_.wt) << std::endl;
        wkm1=w;
    }
    
    else if (params_.damping_method == TRACKING_ERROR)
    {
        double min_singular_value = svd.singularValues()(0);
        for (int i=1; i<jac.rows(); i++) 
        {
            if((svd.singularValues()(i) < min_singular_value) && (svd.singularValues()(i)>params_.eps)) //What is a zero singular value, and what is not. Less than 0.005 seems OK
            {
                min_singular_value = svd.singularValues()(i);
                //file << "Minimum sv:"<< min_singular_value << std::endl;
            }
        }
        damping_factor = pow(min_singular_value,2) * params_.deltaRMax/(1-params_.deltaRMax);
    }
    
    else if (params_.damping_method == SINGULAR_REGION)
    {
        damping_factor = ((S(S.size()-1)>=params_.eps) ? 0 : (sqrt(1-pow(S(S.size()-1)/params_.eps,2))*params_.lambda0)); //The last singular value seems to be less than 0.01 near singular configurations
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
    
    

    int task_size = 1;
    
    //Weighting matrix for endeffector Jacobian Je (jac)
    Eigen::MatrixXd We = Eigen::MatrixXd::Identity(jac.rows(), jac.rows());
    
    //Weighting matrix for additional/task constraints Jc
    Eigen::MatrixXd Wc = Eigen::MatrixXd::Identity(task_size,task_size);
    Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(task_size,jac.columns());
    
    //Weighting matrix for damping 
    Eigen::MatrixXd Wv = Eigen::MatrixXd::Identity(jac.columns(), jac.columns());
    
    Eigen::VectorXd v_in_vec = Eigen::VectorXd::Zero(jac.rows());
    Eigen::VectorXd v_in_vec_base = Eigen::VectorXd::Zero(jac_base.rows());
    Eigen::MatrixXd qdot_out_vec;


    ///use calculated damping value lambda for SVD
    Wv = Wv*damping_factor*damping_factor; //why squared?
    
    ///convert input
    for (int i=0; i<jac.rows(); i++)
    {    
		v_in_vec(i)=v_in(i);    
	}
    
    
    ///// formula from book (2.3.19)
    ////reults in oscillation without task constraints and damping close to 0.0 (when far from singularity)?
    //Eigen::MatrixXd tmp = (jac.data.transpose()*We*jac.data+Jc.transpose()*Wc*Jc+Wv).inverse();
    //qdot_out_vec= tmp*(jac.data.transpose()*We*v_in_vec);
    
    /// formula from book (2.3.14)
    //additional task constraints can not be considered
    Eigen::MatrixXd jac_pinv = Eigen::MatrixXd::Zero(jac.columns(),jac.rows());
    Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(jac.columns(),jac.rows());
    for (int i=0; i<S.rows(); i++)
    {
        for (int j=0; j<jac.rows(); j++)
        {
            for (int k=0; k<jac.columns(); k++)
            {
                double denominator = pow(S(i),2)+pow(damping_factor,2);
                double factor = (denominator < params_.eps) ? 0.0 : S(i)/denominator;
                jac_pinv(k,j)+=factor*svd.matrixV()(k,i)*svd.matrixU()(j,i);
            }
        }
    }
	//std::cout << "Pseudo inverse jacobian:\n " << jac_pinv << "\n";

	qdot_out_vec = jac_pinv*v_in_vec;
		
    ///convert output
    for(int i=0; i<jac.columns(); i++){
		qdot_out(i)=qdot_out_vec(i,0);   
	}
	
	if(DEBUG)
	{
		//compute manipulability
		///kappa = sqrt(norm(J*Jt))
		///see  T.Yoshikawa "Manipulability of robotic mechanisms"
		///     International Journal of Robotics Research, 4(2):3-9, 1985
        //Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac.data * jac.data.transpose();
        //double d = prod.determinant();
        //double kappa = std::sqrt(d);
        //std::cout << "\nManipulability: " << kappa << "\n";
        //ROS_WARN("Damping factor: %f",damping_factor);
        //std::cout<<"task_size:"<<task_size<<std::endl;
        //std::cout << "Current jacobian:\n " << jac.data << "\n";
        //std::cout << "Current We:\n " << We << "\n";
        //std::cout << "Current Wc:\n " << Wc << "\n";
        //std::cout << "Current Wv:\n " << Wv << "\n";
        //std::cout << "Jc:\n " << Jc << "\n";
        //std::cout<<"Singular values"<<svd.singularValues()<<std::endl;
        //std::cout << "Damping factor" << damping_factor << std::endl;
        //std::cout << "Reciprocal Condition number" << 1/(prod.norm()*prod.inverse().norm())<<'\n';
        //std::cout << "DEBUG END" << std::endl;
    }
    
    return 1;
}




