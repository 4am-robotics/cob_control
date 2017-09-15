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


#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_builder.h>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_ramp.h>
#include <cob_cartesian_controller/trajectory_profile_generator/trajectory_profile_generator_sinoid.h>
#include <cob_cartesian_controller/Profile.h>

/* BEGIN TrajectoryProfileBuilder *****************************************************************************************/
TrajectoryProfileBase* TrajectoryProfileBuilder::createProfile(const cob_cartesian_controller::CartesianActionStruct& params)
{
    cob_cartesian_controller::Profile msg;

    const int RAMP = static_cast<const int>(msg.RAMP);
    const int SINOID = static_cast<const int>(msg.SINOID);

    TrajectoryProfileBase* ib = NULL;
    switch (params.profile.profile_type)
    {
        case RAMP:
            ib = new TrajectoryProfileRamp(params);
            break;
        case SINOID:
            ib = new TrajectoryProfileSinoid(params);
            break;
        default:
            ROS_ERROR("Unknown Profile");
            break;
    }

    return ib;
}
/* END TrajectoryProfileBuilder *******************************************************************************************/
