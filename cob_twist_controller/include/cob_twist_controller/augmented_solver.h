#ifndef AUGMENTED_SOLVER_HPP
#define AUGMENTED_SOLVER_HPP

#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/utilities/svd_HH.hpp>
#include <Eigen/LU>
#include <iostream>


/**
* Implementation of a inverse velocity kinematics algorithm based
* on the generalize pseudo inverse to calculate the velocity
* transformation from Cartesian to joint space of a general
* KDL::Chain. It uses a svd-calculation based on householders
* rotations.
*
* @ingroup KinematicFamily
*/
class augmented_solver
{
public:
    /**
     * Constructor of the solver
     *
     * @param chain the chain to calculate the inverse velocity
     * kinematics for
     * @param eps if a singular value is below this value, its
     * inverse is set to zero, default: 0.00001
     * @param maxiter maximum iterations for the svd calculation,
     * default: 150
     *
     */
    augmented_solver(const KDL::Chain& chain, double eps=0.00001, int maxiter=150);
    ~augmented_solver();

    /** CartToJnt for chain NOT including base **/
    virtual int CartToJnt(const KDL::JntArray& q_in, KDL::Twist& v_in, KDL::JntArray& qdot_out);

    /** CartToJnt for chain including base **/
    //virtual int CartToJnt(const KDL::JntArray& q_in, KDL::Twist& v_in, KDL::JntArray& qdot_out, KDL::JntArray& qdot_base_out);
    
    /** not (yet) implemented. */
    virtual int CartToJnt(const KDL::JntArray& q_init, const KDL::FrameVel& v_in, KDL::JntArrayVel& q_out){return -1;};

    //void setAugmentedJacobian(Eigen::Matrix<double,6,Eigen::Dynamic> _jac_augmented);

    //void JLATask(const KDL::JntArray q_in, Eigen::Matrix<double, 10, 1> &z_in, Eigen::Matrix<double, 10 , 10> &jac_c,  Eigen::Matrix<double, 10, 10> &W_c);
    //void ManipulabilityTask(const KDL::JntArray q_in, Eigen::Matrix<double, 10, 1> &z_in, Eigen::Matrix<double, 10 , 10> &jac_c,  Eigen::Matrix<double, 10, 10> &W_c);
    //void BaseObstacleTask(const KDL::JntArray q_in, Eigen::Matrix<double, 10, 1> &z_in, Eigen::Matrix<double, 10 , 10> &jac_c,  Eigen::Matrix<double, 10, 10> &W_c);
    //void setBaseVel(double vel_x, double vel_y, double vel_theta);

    //void setBaseToArmFactor(double base_to_arm_factor){ base_to_arm_factor_ = base_to_arm_factor; }

private:
    const KDL::Chain chain;
    KDL::ChainJntToJacSolver jnt2jac;
    KDL::Jacobian jac;
    KDL::SVD_HH svd;
    std::vector<KDL::JntArray> U;
    KDL::JntArray S;
    std::vector<KDL::JntArray> V;
    KDL::JntArray tmp;
    double eps;
    int maxiter;
    //bool base_is_actived_;
    //double base_to_arm_factor_;

    //double vel_x_;
    //double vel_y_;
    //double vel_theta_;

};
#endif

