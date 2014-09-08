#include "ros/ros.h"

#include "cob_twist_controller/augmented_solver.h"

#define DEBUG true


augmented_solver::augmented_solver(const KDL::Chain& _chain, double _eps, int _maxiter):
    chain(_chain),
    jnt2jac(chain),
    jac(chain.getNrOfJoints()),
    svd(jac),
    U(6,KDL::JntArray(chain.getNrOfJoints())),
    S(chain.getNrOfJoints()),
    V(chain.getNrOfJoints(), KDL::JntArray(chain.getNrOfJoints())),
    tmp(chain.getNrOfJoints()),
    eps(_eps),
    maxiter(_maxiter)
{
    //base_is_actived_ = true;
    //base_to_arm_factor_ = 1.0;
    //vel_x_ = 0.0;
    //vel_y_ = 0.0;
    //vel_theta_ = 0.0;
}

augmented_solver::~augmented_solver()
{
}

//void augmented_solver::setBaseVel(double vel_x, double vel_y, double vel_theta)
//{
    //vel_x_ = vel_x;
    //vel_y_ = vel_y;
    //vel_theta_ = vel_theta;
//}


//void augmented_solver::ManipulabilityTask(const KDL::JntArray q_in, Eigen::Matrix<double, 10, 1> &z_in, Eigen::Matrix<double, 10 , 10> &jac_c,  Eigen::Matrix<double, 10, 10>  &W_c)
//{
    /////Definition of Manipulability additional task 
    /////Define z_in, jac_c and W_c for inverse kinematic calculation 
    /////Parameters: ?? 

    //jnt2jac.JntToJac(q_in,jac);
    //Eigen::Matrix<double,6,6> prod = jac.data * jac.data.transpose();
    //float d = prod.determinant();
    ////std::cerr << "Current Manipulability: " << d << "\n";

    //Eigen::MatrixXd NULLSPACE = jac.data.fullPivLu().kernel();
//}


//void augmented_solver::BaseObstacleTask(const KDL::JntArray q_in, Eigen::Matrix<double, 10, 1> &z_in, Eigen::Matrix<double, 10 , 10> &jac_c,  Eigen::Matrix<double, 10, 10>  &W_c)
//{
    /////Definition of BaseObstacle additional task ###
    /////Define z_in, jac_c and W_c for inverse kinematic calculation
    /////Parameters: ??
    
    ////additional jacobian
    //int i = 7;
    //z_in(i,0) = vel_x_;
    //jac_c(i,i) = 1;
    //W_c(i,i) = 0.01;
//}


//void augmented_solver::JLATask(const KDL::JntArray q_in, Eigen::Matrix<double, 10, 1> &z_in, Eigen::Matrix<double, 10 , 10> &jac_c,  Eigen::Matrix<double, 10, 10>  &W_c)
//{
    /////Definition of JLA additional task ###
    /////Define z_in, jac_c and W_c for inverse kinematic calculation
    /////Parameters: 
        /////q_max (max joint limit for each manipulator joint)
        /////tau (begin avoidance tau rad from joint limit)
        /////W_0 (force of avoidance)

    /////TODO:
        /////* read Parameters out of yaml file
        /////* read Joint limits out of urdf
        /////* Tune parameters
        /////* Test, test, test

    //std::vector<double> q_max;
    //q_max.push_back(1.1); //Joint 0
    //q_max.push_back(1.1); //Joint 1
    //q_max.push_back(1.1); //Joint 2
    //q_max.push_back(1.1); //Joint 3
    //q_max.push_back(1.1); //Joint 4
    //q_max.push_back(1.1); //Joint 5
    //q_max.push_back(1.1); //Joint 6

    //double tau = 0.1;
    //double W_0 = 0.01;

    //// End Parameters


    //for(unsigned int i = 0; i < 7; i++)
    //{
        //if(q_in(i) <= 0.0)
            //z_in(i,0) = -1 * q_max.at(i);
        //else
            //z_in(i,0) = 1 * q_max.at(i);
    //}
    
    ////additional jacobian
    //for(unsigned int i=0 ; i<7 ; i++)
        //jac_c(i,i) = 1;

    ////weighting of JLA
    //for(unsigned int i=0 ; i<7 ; i++)
    //{
        //if(fabs(q_in(i)-z_in(i,0)) > tau)
        //{
            //W_c(i,i) = 0.0;
        //}
        //else
        //{
            //std::cerr << "near joint limit at joint: " << i << " val: " << fabs(q_in(i)-z_in(i,0)) << "\n"; 
            //if(fabs(q_in(i)-z_in(i,0)) < 0.0)       
            //{
                //W_c(i,i) = W_0/4*(1+cos(3.14*(fabs(q_in(i)-z_in(i,0))/tau)));        
            //}
            //else
            //{
                //W_c(i,i) = W_0/2;
            //}
        //}
    //}
//}


//int augmented_solver::CartToJnt(const KDL::JntArray& q_in, KDL::Twist& v_in, KDL::JntArray& qdot_out, KDL::JntArray& qdot_base_out)
//{
    //double damping_factor = 0.01;

    ////Let the ChainJntToJacSolver calculate the jacobian "jac" for
    ////the current joint positions "q_in"
    //jnt2jac.JntToJac(q_in,jac);

    //////v_in.vel.x(0.0);
    //////v_in.vel.y(0.0);
    //////v_in.vel.z(-0.02);
    //////v_in.rot.x(0.0);
    //////v_in.rot.y(0.0);
    //////v_in.rot.z(0.0);

    ////Create standard platform jacobian
    //Eigen::Matrix<double,6,3> jac_base;
    //jac_base.setZero();
    //if(base_is_actived_)
    //{
        //jac_base(0,0) = base_to_arm_factor_;
        //jac_base(1,1) = base_to_arm_factor_;
        //jac_base(5,2) = base_to_arm_factor_;
    //}

    ////Put full jacobian matrix together
    //Eigen::Matrix<double, 6, Eigen::Dynamic> jac_full;
    //jac_full.resize(6,chain.getNrOfJoints() + jac_base.cols());
    //jac_full << jac.data, jac_base;
    //int num_dof = chain.getNrOfJoints() + jac_base.cols();
    //if(DEBUG)
        //std::cout << "Combined jacobian:\n " << jac_full << "\n";

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

    ////Do a singular value decomposition of "jac" with maximum
    ////iterations "maxiter", put the results in "U", "S" and "V"
    ////jac = U*S*Vt
    //int ret = svd.calculate(jac,U,S,V,maxiter);

    //double sum;
    //unsigned int i,j;

    //// We have to calculate qdot_out = jac_pinv*v_in
    //// Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
    //// qdot_out = V*S_pinv*Ut*v_in

    ////first we calculate Ut*v_in
    //for (i=0;i<jac.columns();i++) 
    //{
        //sum = 0.0;
        //for (j=0;j<jac.rows();j++) 
        //{
            //sum+= U[j](i)*v_in(j);
        //}
        ////If the singular value is too small (<eps), don't invert it but
        ////set the inverted singular value to zero (truncated svd)
        //tmp(i) = sum*(fabs(S(i))<eps?0.0:1.0/S(i));
        ////tmp(i) = sum*1.0/S(i);
    //}
    ////tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
    ////it with V to get qdot_out
    //for (i=0;i<jac.columns();i++) 
    //{
        //sum = 0.0;
        //for (j=0;j<jac.columns();j++)
        //{
            //sum+=V[i](j)*tmp(j);
        //}
        ////Put the result in qdot_out
        //qdot_out(i)=sum;
    //}
    //if(DEBUG)
        //std::cout << "Solution SVD: " << qdot_out(0) << " " << qdot_out(1) << " " << qdot_out(2) << " " << qdot_out(3) << " " << qdot_out(4) << " " << qdot_out(5) << " " << qdot_out(6)  << "\n====\n";
    ////return the return value of the svd decomposition
    ////New calculation
    //for(unsigned int i=0;i<7;i++)
    //{
        //qdot_out(i)=q_dot_conf_control(i,0);
    //}
    //if(base_is_actived_)
    //{
        //for(unsigned int i = 7; i<7+3; i++)
        //{
            //qdot_base_out(i-7) = q_dot_conf_control(i,0);
        //}
    //}

    //if(DEBUG)
        //std::cout << "Solution ConfControl: " << qdot_out(0) << " " << qdot_out(1) << " " << qdot_out(2) << " " << qdot_out(3) << " " << qdot_out(4) << " " << qdot_out(5) << " " << qdot_out(6)  << "\n====\n";
    
    //return ret;
//}




int augmented_solver::CartToJnt(const KDL::JntArray& q_in, KDL::Twist& v_in, KDL::JntArray& qdot_out)
{
    int ret = -1;
    int num_dof = chain.getNrOfJoints();

    ///Let the ChainJntToJacSolver calculate the jacobian "jac" for the current joint positions "q_in"
    jnt2jac.JntToJac(q_in,jac);
    if(DEBUG)
        std::cout << "Current jacobian:\n " << jac.data << "\n";

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

    
    /// Do a singular value decomposition of "jac" with maximum 
    /// iterations "maxiter", put the results in "U", "S" and "V"
    /// jac = U*S*Vt
    /// jac: mxn
    /// U: mxm
    /// S: nx1 (vector of singular values) it actually is a nxm with only diagonal entries
    /// Vt: nxn
    /// qdot_out: nx1
    /// v_in: mx1
    ret = svd.calculate(jac,U,S,V,maxiter);
    if(DEBUG)
        std::cout << "Singular Values:\n " << S.data << "\n";
        
    if(DEBUG)
    {
        ///compute manipulability
        ///kappa = sqrt(norm(J*Jt)) 
        ///see  T.Yoshikawa "Manipulability of robotic mechanisms"
        ///     International Journal of Robotics Research, 4(2):3-9, 1985
        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = jac.data * jac.data.transpose();
        double d = prod.determinant();
        double kappa = std::sqrt(d);
        
        std::cout << "\nManipulability: " << kappa << "\n";
    }
    

    /// We have to calculate qdot_out = jac_pinv*v_in
    /// Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
    /// qdot_out = V*S_pinv*Ut*v_in

    double sum;
    unsigned int i,j;

    ///first we calculate Ut*v_in
    for (i=0;i<jac.columns();i++) //n
    {
        sum = 0.0;
        for (j=0;j<jac.rows();j++) //m
        {
            sum+= U[j](i)*v_in(j);
        }
        tmp(i) = sum;
    }
    
    ///next is S^-1*Ut*v_in
    for (i=0;i<jac.columns();i++) //n
    {
        /////exact solution
        ////tmp(i) = tmp(i)*1.0/S(i);
        ///truncated svd
        ///If the singular value is too small (<eps), don't invert it but set the inverted singular value to zero 
        tmp(i) = tmp(i)*(fabs(S(i))<eps?0.0:1.0/S(i));
    }
    ///tmp is now: tmp=S_pinv*Ut*v_in, 
    
    ///we still have to left-multiply it with V to get qdot_out
    for (i=0;i<jac.columns();i++) //n
    {
        sum = 0.0;
        for (j=0;j<jac.columns();j++) //n
        {
            sum+=V[i](j)*tmp(j);
        }
        ///Put the result in qdot_out
        qdot_out(i)=sum;
    }
    
    return ret;
}




