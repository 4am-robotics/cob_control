/*
 * Copyright 2020 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
