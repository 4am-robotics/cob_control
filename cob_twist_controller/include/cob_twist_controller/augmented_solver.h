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

struct AugmentedSolverParams {
    int damping_method;
    double eps;
    double damping_factor;
    double lambda0;
    double wt;
    bool JLA_active;
    bool enforce_limits;
    double tolerance;
    bool base_compensation;
    bool base_active;
    double base_ratio;

};

enum DampingMethodTypes {
    MANIPULABILITY = 0,
    CONSTANT = 1,
    TRUNCATION = 2
};



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
    augmented_solver(const KDL::Chain& chain, double eps=0.001, int maxiter=5);
    ~augmented_solver();
    
    /** CartToJnt for chain using SVD including base and various DampingMethods **/
    virtual int CartToJnt(const KDL::JntArray& q_in, const KDL::JntArray& last_q_dot, KDL::Twist& v_in, KDL::JntArray& qdot_out, std::vector<float> limits_min, std::vector<float> limits_max, KDL::Frame &base_position, KDL::Frame &chain_base);

    inline virtual int CartToJnt(const KDL::JntArray& q_in, const KDL::JntArray& last_q_dot, KDL::Twist& v_in, KDL::JntArray& qdot_out, std::vector<float> limits_min, std::vector<float> limits_max)
    {
        KDL::Frame dummy;
        dummy.p = KDL::Vector(0,0,0);
        dummy.M = KDL::Rotation::Quaternion(0,0,0,0);
        
        return CartToJnt(q_in, last_q_dot, v_in, qdot_out, limits_min, limits_max, dummy, dummy);
    }

    /** not (yet) implemented. */
    virtual int CartToJnt(const KDL::JntArray& q_init, const KDL::JntArray& last_q_dot, const KDL::FrameVel& v_in, KDL::JntArrayVel& q_out){return -1;};
    
    void SetAugmentedSolverParams(AugmentedSolverParams params){params_ = params;}
    AugmentedSolverParams GetAugmentedSolverParams(){return params_;}

private:
    const KDL::Chain chain_;
    KDL::Jacobian jac_, jac_base_;
    KDL::ChainJntToJacSolver jnt2jac_;
    int maxiter_;
    double x_,y_,r_,z_;
    
    AugmentedSolverParams params_;
    Eigen::VectorXd calculate_weighting(const KDL::JntArray& q, const KDL::JntArray& last_q_dout,  std::vector<float> limits_min, std::vector<float> limits_max);
    Eigen::VectorXd enforce_limits(const KDL::JntArray& q, Eigen::MatrixXd qdout_out, std::vector<float> limits_min, std::vector<float> limits_max);
};
#endif
