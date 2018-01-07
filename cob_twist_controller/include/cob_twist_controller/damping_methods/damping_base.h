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


#ifndef COB_TWIST_CONTROLLER_DAMPING_METHODS_DAMPING_BASE_H
#define COB_TWIST_CONTROLLER_DAMPING_METHODS_DAMPING_BASE_H

#include "cob_twist_controller/cob_twist_controller_data_types.h"

/// Base class for solvers, defining interface methods.
class DampingBase
{
    public:
        explicit DampingBase(const TwistControllerParams& params) : params_(params)
        {}

        virtual ~DampingBase() {}

        virtual Eigen::MatrixXd getDampingFactor(const Eigen::VectorXd& sorted_singular_values,
                                        const Eigen::MatrixXd& jacobian_data) const = 0;

    protected:
        const TwistControllerParams params_;
};

#endif  // COB_TWIST_CONTROLLER_DAMPING_METHODS_DAMPING_BASE_H
