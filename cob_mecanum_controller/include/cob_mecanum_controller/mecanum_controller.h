#pragma once

#include <eigen3/Eigen/Dense>
namespace cob_mecanum_controller
{
class MecanumController
{
public:
  MecanumController(double lx, double ly, double r);

  Eigen::Vector3d wheelToTwist(Eigen::Vector4d wheel_velocity);

  Eigen::Vector4d twistToWheel(Eigen::Vector3d twist);

protected:
  Eigen::Matrix<double, 4, 3> forward_matrix_;
  Eigen::Matrix<double, 3, 4> inverse_matrix_;
  double r_;
};

}  // namespace cob_mecanum_controller
