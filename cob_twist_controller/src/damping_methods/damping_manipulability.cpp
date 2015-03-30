/*
 * DampingManipulability.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: fxm-mb
 */

#include "cob_twist_controller/damping_methods/damping_manipulability.h"

DampingManipulability::DampingManipulability(AugmentedSolverParams &asSolverParams, Matrix6Xd &jacobianData)
    : DampingBase(asSolverParams, jacobianData)
{

}

double DampingManipulability::get_damping_factor() const
{
    double wt = this->asSolverParams_.wt;
    double lambda0 = this->asSolverParams_.lambda0;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> prod = this->jacobianData_ * this->jacobianData_.transpose();
    double d = prod.determinant();
    double w = std::sqrt(std::abs(d));
    double damping_factor;
    if (w < wt || 0.0 == wt) // added safety to avoid DIV/0
    {
        damping_factor = 0.0;
    }
    else
    {
        double tmp_w = (1 - w / wt);
        damping_factor = lambda0 * tmp_w * tmp_w;
    }

    return damping_factor;
}
