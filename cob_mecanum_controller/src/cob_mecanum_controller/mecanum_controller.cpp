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

#include <cob_mecanum_controller/mecanum_controller.h>

// Maths from https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

namespace cob_mecanum_controller
{
MecanumController::MecanumController(double lx, double ly, double r) : r_(r)
{
  double lxy = lx + ly;
  double lxy_inv = 1.0 / lxy;
  inverse_matrix_ << 1, 1, 1, 1, -1, 1, 1, -1, -lxy_inv, lxy_inv, -lxy_inv, lxy_inv;
  forward_matrix_ << 1, -1, -lxy, 1, 1, lxy, 1, 1, -lxy, 1, -1, lxy;
}

Eigen::Vector3d MecanumController::wheelToTwist(Eigen::Vector4d wheel_velocity)
{
  wheel_velocity(1) = wheel_velocity(1) * -1;
  wheel_velocity(3) = wheel_velocity(3) * -1;
  return r_ / 4 * inverse_matrix_ * wheel_velocity;
}

Eigen::Vector4d MecanumController::twistToWheel(Eigen::Vector3d twist)
{
  Eigen::Vector4d wheel_cmd = 1.0 / r_ * forward_matrix_ * twist;
  wheel_cmd(1) = wheel_cmd(1) * -1;
  wheel_cmd(3) = wheel_cmd(3) * -1;
  return wheel_cmd;
}

}  // namespace cob_mecanum_controller
