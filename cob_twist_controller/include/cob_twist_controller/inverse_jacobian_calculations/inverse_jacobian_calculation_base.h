/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
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


#ifndef COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_BASE_H
#define COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_BASE_H

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include "cob_twist_controller/damping_methods/damping_base.h"
#include "cob_twist_controller/cob_twist_controller_data_types.h"

class IPseudoinverseCalculator
{
    public:
        /**
         * Pure virtual method for calculation of the pseudoinverse
         * @param jacobian The Jacobi matrix.
         * @return A pseudoinverse Jacobian
         */
        virtual Eigen::MatrixXd calculate(const Eigen::MatrixXd& jacobian) const = 0;

        /**
         * Pure virtual method for calculation of the pseudoinverse (allows to consider damping and truncation)
         * @param params The parameters from parameter server.
         * @param db The damping method.
         * @param jacobian The Jacobi matrix.
         * @return A pseudoinverse Jacobian
         */
        virtual Eigen::MatrixXd calculate(const TwistControllerParams& params,
                                          boost::shared_ptr<DampingBase> db,
                                          const Eigen::MatrixXd& jacobian) const = 0;

        /**
         * Class has no members so implementing an empty destructor.
         */
        virtual ~IPseudoinverseCalculator() {}
};

#endif  // COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_BASE_H
