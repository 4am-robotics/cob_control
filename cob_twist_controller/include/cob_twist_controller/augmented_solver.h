#ifndef AUGMENTED_SOLVER_HPP
#define AUGMENTED_SOLVER_HPP

#include <math.h>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD> 
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
    
    /** CartToJnt for chain NOT including base using truncated SVD **/
    virtual int CartToJnt(const KDL::JntArray& q_in, KDL::Twist& v_in, KDL::JntArray& qdot_out);
    /** CartToJnt for chain NOT including base using SVD with Damping **/
    virtual int CartToJnt(const KDL::JntArray& q_in, KDL::Twist& v_in, KDL::JntArray& qdot_out, std::string damping_method);
    
    /** not (yet) implemented. */
    virtual int CartToJnt(const KDL::JntArray& q_init, const KDL::FrameVel& v_in, KDL::JntArrayVel& q_out){return -1;};

private:
    const KDL::Chain chain;
    KDL::Jacobian jac;
    KDL::ChainJntToJacSolver jnt2jac;
    double eps;
    int maxiter;
    Eigen::MatrixXd Jcm1;
    double wkm1;
    bool initial_iteration;
};
#endif

