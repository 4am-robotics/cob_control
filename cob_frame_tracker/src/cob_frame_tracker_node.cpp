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


#include <ros/ros.h>
#include <cob_frame_tracker/cob_frame_tracker.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cob_frame_tracker_node");
    CobFrameTracker *cft = new CobFrameTracker();

    if (!cft->initialize())
    {
        ROS_ERROR("Failed to initialize FrameTracker");
        return -1;
    }

    ros::spin();
    delete cft;
    return 0;
}
