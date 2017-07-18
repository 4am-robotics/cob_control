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


#ifndef COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_SINOID_H
#define COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_SINOID_H

#include <vector>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_base.h>

/* BEGIN TrajectoryProfileSinoid ****************************************************************************************/
class TrajectoryProfileSinoid : public TrajectoryProfileBase
{
public:
    explicit TrajectoryProfileSinoid(const cob_cartesian_controller::CartesianActionStruct& params)
    :    TrajectoryProfileBase(params)
    {}

    ~TrajectoryProfileSinoid()
    {}

    virtual bool getProfileTimings(double Se, double te, bool calcMaxTe, cob_cartesian_controller::ProfileTimings& pt);
    virtual std::vector<double> getTrajectory(double se, cob_cartesian_controller::ProfileTimings pt);
};
/* END TrajectoryProfileSinoid **********************************************************************************************/

#endif  // COB_CARTESIAN_CONTROLLER_TRAJECTORY_PROFILE_GENERATOR_TRAJECTORY_PROFILE_GENERATOR_SINOID_H
