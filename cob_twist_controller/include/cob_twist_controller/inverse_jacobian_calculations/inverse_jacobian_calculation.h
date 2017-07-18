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


#ifndef COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_H
#define COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_H

#include "cob_twist_controller/inverse_jacobian_calculations/inverse_jacobian_calculation_base.h"

/* BEGIN PInvBySVD **********************************************************************************************/
class PInvBySVD : public IPseudoinverseCalculator
{
    public:
        /** Implementation of calculate member
         * See base for more information on parameters
         */
        virtual Eigen::MatrixXd calculate(const Eigen::MatrixXd& jacobian) const;

        /** Implementation of calculate member
         * See base for more information on parameters
         */
        virtual Eigen::MatrixXd calculate(const TwistControllerParams& params,
                                          boost::shared_ptr<DampingBase> db,
                                          const Eigen::MatrixXd& jacobian) const;

        virtual ~PInvBySVD() {}
};
/* END PInvBySVD ************************************************************************************************/

/* BEGIN PInvDirect **********************************************************************************************/
class PInvDirect : public IPseudoinverseCalculator
{
    public:
        /** Implementation of calculate member
         * See base for more information on parameters
         */
        virtual Eigen::MatrixXd calculate(const Eigen::MatrixXd& jacobian) const;

        /** Implementation of calculate member
         * See base for more information on parameters
         */
        virtual Eigen::MatrixXd calculate(const TwistControllerParams& params,
                                          boost::shared_ptr<DampingBase> db,
                                          const Eigen::MatrixXd& jacobian) const;

        virtual ~PInvDirect() {}
};
/* END PInvDirect ************************************************************************************************/

#endif  // COB_TWIST_CONTROLLER_INVERSE_JACOBIAN_CALCULATIONS_INVERSE_JACOBIAN_CALCULATION_H
