#include "ros/ros.h"

#include "cob_twist_controller/augmented_solver.h"

#define DEBUG true


augmented_solver::augmented_solver(const KDL::Chain& _chain, double _eps, int _maxiter):
    chain(_chain),
    jac(chain.getNrOfJoints()),
    jnt2jac(chain),
    eps(_eps),
    maxiter(_maxiter)
{}

augmented_solver::~augmented_solver()
{}


int augmented_solver::CartToJnt(const KDL::JntArray& q_in, KDL::Twist& v_in, KDL::JntArray& qdot_out)
{
    ///Let the ChainJntToJacSolver calculate the jacobian "jac" for the current joint positions "q_in"
    jnt2jac.JntToJac(q_in,jac);
    
    ///Use Eigen::JacobiSVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac.data, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd S = svd.singularValues();
    Eigen::VectorXd v_in_vec = Eigen::VectorXd::Zero(jac.rows());
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(jac.columns(),jac.rows());
    
    ///truncated S(i)
    for (int i=0; i<(jac.rows()<jac.columns()?jac.rows():jac.columns());i++)
    {    S_inv(i,i)=((S(i)<eps)?0:1/S(i));    }
    
    ///convert input
    for (int i=0; i<jac.rows(); i++)
    {    v_in_vec(i)=v_in(i);    }
    
    Eigen::MatrixXd qdot_out_vec= svd.matrixV()*S_inv*svd.matrixU().transpose()*v_in_vec;
    
    ///convert output
    for(int i=0; i<jac.columns(); i++)
    {    qdot_out(i)=qdot_out_vec(i,0);    }
    
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
        
        std::cout << "Singular Values:\n " << S << "\n";
        std::cout << "Singular Values inv:\n " << S_inv << "\n";
        std::cout << "Result:\n " << qdot_out_vec << "\n";
    }
    
    return 1;
}




