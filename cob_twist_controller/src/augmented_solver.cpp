#include "ros/ros.h"

#include "cob_twist_controller/augmented_solver.h"
#include <fstream>


#define DEBUG true

augmented_solver::augmented_solver(const KDL::Chain& _chain, double _eps):
    chain(_chain),
    jnt2jac(chain),
    jac(chain.getNrOfJoints()),        
    eps(_eps),
    task_size(1),    
    We(),   
    Wc(),
    Wv(),
    Jc(),
    Jcm1(),
    damping_factor(),
    wkm1(0),
    firstIterationDone(false),
    quasiZero(0.000000001)
{	
    
}


augmented_solver::~augmented_solver()
{
}

int augmented_solver::CartToJnt(const KDL::JntArray& q_in, KDL::Twist& v_in, KDL::JntArray& qdot_out)
{
	std::ofstream file("test_end_effect.txt", std::ofstream::app);
	
	int ret = -1;
    int num_dof = chain.getNrOfJoints();
    Eigen::MatrixXd qdot_out_vec;

    ///Let the ChainJntToJacSolver calculate the jacobian "jac" for the current joint positions "q_in"
    jnt2jac.JntToJac(q_in,jac);
            
    Eigen::MatrixXd We = Eigen::MatrixXd::Identity(jac.rows(), jac.rows());
    Eigen::MatrixXd Wc = Eigen::MatrixXd::Zero(task_size,task_size);
    Eigen::MatrixXd Wv = Eigen::MatrixXd::Identity(jac.columns(), jac.columns());
    Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(task_size,jac.columns());   

	std::string dampingFactorSelection = "manipulability";

    ///Testing:
    //v_in.vel.x(0.0);
    //v_in.vel.y(0.0);
    //v_in.vel.z(-0.02);
    //v_in.rot.x(0.0);
    //v_in.rot.y(0.0);
    //v_in.rot.z(0.0);


    //double damping_factor = 0.01;
    
    ////Weighting Matrices
    //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> W_v;
    //W_v.resize(num_dof,num_dof);
    //W_v.setZero();

    //for(int i=0 ; i<num_dof ; i++)
        //W_v(i,i) = damping_factor;


    //Eigen::Matrix<double, 6,6> W_e;
    //W_e.setZero();
    //for(unsigned int i=0 ; i<6 ; i++)
        //W_e(i,i) = 1;

    //if(DEBUG)
        //std::cout << "Weight matrix defined\n";
    ////W_e.setIdentity(6,6);


    ////Matrices for additional tasks
    //Eigen::Matrix<double, 10, 1> z_in;
    //z_in.setZero();
    //Eigen::Matrix<double, 10 , 10> jac_c;
    //jac_c.setZero();        
    //Eigen::Matrix<double, 10, 10> W_c;
    //W_c.setZero();

    ////JLATask(q_in, z_in, jac_c, W_c);
    ////ManipulabilityTask(q_in, z_in, jac_c, W_c);
    ////BaseObstacleTask(q_in, z_in, jac_c, W_c);


    ////Inversion
    //// qdot_out = (jac_full^T * W_e * jac_full + jac_augmented^T * W_c * jac_augmented + W_v)^-1(jac_full^T * W_e * v_in + jac_augmented^T * W_c * z_in)
    //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> damped_inversion;
    //damped_inversion.resize(num_dof,num_dof);

    ////damped_inversion = (jac_full.transpose() * W_e * jac_full) +  (jac_c.transpose() * W_c * jac_c) + W_v;
    //damped_inversion = (jac_full.transpose() * W_e * jac_full) + W_v;
    //if(DEBUG)
        //std::cout << "Inversion done\n";

    //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> q_dot_conf_control;
    //Eigen::Matrix<double, 6, 1> v_in_eigen;
    //v_in_eigen.setZero();
    //v_in_eigen(0,0) = v_in.vel.x();
    //v_in_eigen(1,0) = v_in.vel.y();
    //v_in_eigen(2,0) = v_in.vel.z();
    //v_in_eigen(3,0) = v_in.rot.x();
    //v_in_eigen(4,0) = v_in.rot.y();
    //v_in_eigen(5,0) = v_in.rot.z();
    ////q_dot_conf_control = damped_inversion.inverse() * ((jac_full.transpose() * W_e * v_in_eigen) + (jac_c.transpose() * W_c * z_in));
    //q_dot_conf_control = damped_inversion.inverse() * jac_full.transpose() * W_e * v_in_eigen;

    //if(DEBUG)
        //std::cout << "Endergebnis: \n" << q_dot_conf_control << "\n";
	
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac.data,Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    Eigen::VectorXd v_in_vec = Eigen::VectorXd::Zero(jac.rows());
    
    //Eigen::VectorXd S = svd.singularValues();
    //
    
    //
    //Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(jac.columns(),jac.rows());
    
    //for (int i=0; i<(jac.rows()<jac.columns()?jac.rows():jac.columns());i++)
		//S_inv(i,i)=(S(i)<eps?0:1/S(i));	
	
	
	 if (dampingFactorSelection=="manipulability")
	{	
		Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac.data * jac.data.transpose();
        double d = prod.determinant();        
        double w = std::sqrt(std::abs(d));      
		double lambda0 = 0.1;
		double wt = 0.001;
		damping_factor = (w<wt ? lambda0 * pow((1 - w/wt),2) : 0);
		//std::cout << "w" << w << " wt" <<wt << " Condicion" << (bool)(w<wt) << "\n";
	}
	
	else if (dampingFactorSelection=="manipulabilityRate")
	{
		Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac.data * jac.data.transpose();
        double d = prod.determinant();        
        double w = std::sqrt(std::abs(d));
		double wt = 0.05;
		double lambda0 = 0.008;
		if (!firstIterationDone)
			damping_factor=lambda0;
		else
			if(w<quasiZero && wkm1<quasiZero)				
				damping_factor = lambda0;
			else
				damping_factor = (w/wkm1 < wt ? lambda0 * (1-w/wkm1) : 0);	
		
		if (DEBUG)
		{			
			std::cout<<"w: "<<w<<" wk-1: "<< wkm1 << " condicion:" << (bool)(w/wkm1 <wt) << std::endl;
		}				
		wkm1=w;		
	}
	
	else if (dampingFactorSelection == "trackingError")	
	{
		double deltaRMax = 0.05;
		damping_factor = (svd.singularValues())(0);
		for (int i=1; i<jac.rows();i++) {					
			if(damping_factor>(svd.singularValues())(i) && ((svd.singularValues())(i))>=0.001)		{
				damping_factor = (svd.singularValues())(i);			
			//file << "Minimum sv:"<< damping_factor << std::endl;
			}
			}		
				
		damping_factor=pow(damping_factor,2)*deltaRMax/(1-deltaRMax);	
	}
	
	else if (dampingFactorSelection == "none")
	{
		Eigen::VectorXd S = svd.singularValues();
		Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(jac.columns(),jac.rows());
    
		for (int i=0; i<(jac.rows()<jac.columns()?jac.rows():jac.columns());i++)
			S_inv(i,i)=(S(i)<eps?0:1/S(i));
			
		for (int i=0; i<jac.rows(); i++)
			v_in_vec(i)=v_in(i);
    
		qdot_out_vec= svd.matrixV()*S_inv*svd.matrixU().transpose()*v_in_vec;
	}
	
	else if (dampingFactorSelection == "singularRegion")
	{
		Eigen::VectorXd S = svd.singularValues();		
		double lambda0 = 0.01;		
		damping_factor = S(S.size()-1)>=eps ? 0 : sqrt(1-pow(S(S.size()-1)/eps,2))*lambda0;		
	}
	
	if(dampingFactorSelection != "none")
	{
		Wv=Wv*damping_factor*damping_factor;
		Eigen::MatrixXd tmp = (jac.data.transpose()*We*jac.data+Jc.transpose()*Wc*Jc+Wv).inverse();	
		if(isnan(tmp.determinant()))
			return -1;
			
		for (int i=0; i<jac.rows(); i++)
		{		
			v_in_vec(i)=v_in(i);		
		}
		if(damping_factor==0) {		
		Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac.data.transpose() * jac.data;
		Eigen::JacobiSVD<Eigen::MatrixXd> prod_svd(prod,Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::VectorXd prod_S = prod_svd.singularValues();
		Eigen::MatrixXd prod_S_inv = Eigen::MatrixXd::Zero(jac.columns(),jac.columns());
    //
		for (int i=0; i<(jac.rows()<jac.columns()?jac.rows():jac.columns());i++)
			prod_S_inv(i,i)=(prod_S(i)==0?0:1/prod_S(i));			
		
		qdot_out_vec= prod_svd.matrixV()*prod_S_inv*prod_svd.matrixU().transpose()*(jac.data.transpose()*We*v_in_vec);		
	}
		else
		
		qdot_out_vec= tmp*(jac.data.transpose()*We*v_in_vec);		
	}
	
	for(int i=0; i<jac.columns(); i++)
		{	
			if(std::abs(qdot_out_vec(i,0))>500) {				
				if (file.is_open())
				  {
					Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac.data * jac.data.transpose();					
					
					double d = prod.determinant();
					double kappa = std::sqrt(d);        					
					file << "Here is the Jacobian:\n" << jac.data << '\n';
					file << "Last Jacobian:\n" << Jcm1 << '\n';
					file << "Here is prod:\n" << prod << '\n';					
					file << "And here is the other stuff:"<< std::endl << "damping_factor" <<damping_factor<<std::endl;
					file << "Singular values:\n" << svd.singularValues();					
					file << "\nManipulability: " << kappa << "\n";
					file<<"El twist deberia ser:\n"<<jac.data*qdot_out_vec<<std::endl;
					file << "La entrada fue: \n" << v_in_vec<<std::endl;
					file << "La salida es: \n" << qdot_out_vec <<'\n';
					file << "inv(jac' jac):\n" << (jac.data.transpose()*jac.data).inverse();
				  }
				  break;
			  }			
		}
		
	for(int i=0; i<jac.columns(); i++)		
		qdot_out(i)=qdot_out_vec(i,0);			
		
	//for (int i=0;i<jac.rows();i++)
		//for (int j=0;j<jac.columns();j++)
			//ROS_WARN("tmp i %d j %d val %f",i,j,tmp(i,j));
			//
	//ROS_WARN("Det tmp: %f",tmp.determinant());
	    
    /// Do a singular value decomposition of "jac" with maximum 
    /// iterations "maxiter", put the results in "U", "S" and "V"
    /// jac = U*S*Vt
    /// jac: mxn
    /// U: mxm
    /// S: nx1 (vector of singular values) it actually is a nxm with only diagonal entries
    /// Vt: nxn
    /// qdot_out: nx1
    /// v_in: mx1
    
    /// We have to calculate qdot_out = jac_pinv*v_in
    /// Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
    /// qdot_out = V*S_pinv*Ut*v_in
        
        
    if(DEBUG)
    {
        //compute manipulability
        ///kappa = sqrt(norm(J*Jt)) 
        ///see  T.Yoshikawa "Manipulability of robotic mechanisms"
        ///     International Journal of Robotics Research, 4(2):3-9, 1985
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac.data * jac.data.transpose();
        double d = prod.determinant();
        double kappa = std::sqrt(d);        
        std::cout << "\nManipulability: " << kappa << "\n";        
		//ROS_WARN("Damping factor: %f",damping_factor);
        //std::cout<<"task_size:"<<task_size<<std::endl;
        std::cout << "Current jacobian:\n " << jac.data << "\n";
        //std::cout << "Current We:\n " << We << "\n";
        //std::cout << "Current Wc:\n " << Wc << "\n";
        //std::cout << "Current Wv:\n " << Wv << "\n";        
        //std::cout << "Jc:\n " << Jc << "\n";
        //std::cout<<"Singular values"<<svd.singularValues()<<std::endl;
        //std::cout << "Damping factor" << damping_factor << std::endl;
        std::cout << "Reciprocal Condition number" << 1/(prod.norm()*prod.inverse().norm())<<'\n';
        //std::cout << "DEBUG END" << std::endl;
    }
    
    Jcm1=jac.data;
    
    ret=1;
    
    if(!firstIterationDone)
		firstIterationDone=true;
		
    return ret;
}




