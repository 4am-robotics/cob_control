/*
 * constraint_params.h
 *
 *  Created on: Jun 8, 2015
 *      Author: fxm-mb
 */

#ifndef CONSTRAINT_PARAMS_H_
#define CONSTRAINT_PARAMS_H_

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>

#include "cob_twist_controller/augmented_solver_data_types.h"



struct Distance
{
    double min_distance;
    Eigen::Vector3d distance_vec;
    std::string frame_name;
};

class ConstraintParamsBase
{
    public:

        ConstraintParamsBase(const AugmentedSolverParams& params) : params_(params)
        {}

        ~ConstraintParamsBase()
        {}

        const AugmentedSolverParams& getAugmentedSolverParams() const
        {
            ROS_INFO_STREAM("AugmentedSolverParams");

            return this->params_;
        }

    protected:
        const AugmentedSolverParams& params_;
};

/* BEGIN ConstraintParamsCA *************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsCA : public ConstraintParamsBase
{
    public:

        Distance current_distance_;

        ConstraintParamsCA(const AugmentedSolverParams& params)
        : ConstraintParamsBase(params)
        {}

        ConstraintParamsCA(const ConstraintParamsCA& cpca)
        : ConstraintParamsBase(cpca.params_)
        {
            ROS_INFO_STREAM("Called copy constructor: " << params_.constraint);
            current_distance_ = cpca.current_distance_;
        }

        virtual ~ConstraintParamsCA()
        {}

};
/* END ConstraintParamsCA ***************************************************************************************/

/* BEGIN ConstraintParamsJLA ************************************************************************************/
/// Class that represents the parameters for the Collision Avoidance constraint.
class ConstraintParamsJLA : public ConstraintParamsBase
{
    public:

        ConstraintParamsJLA(const AugmentedSolverParams& params)
        : ConstraintParamsBase(params)
        {}

        ConstraintParamsJLA(const ConstraintParamsJLA& cpjla)
        : ConstraintParamsBase(cpjla.params_)
        {}

        virtual ~ConstraintParamsJLA()
        {}

};
/* END ConstraintParamsJLA **************************************************************************************/

typedef boost::shared_ptr<ConstraintParamsBase> tConstraintParamsBase;

#endif /* CONSTRAINT_PARAMS_H_ */
