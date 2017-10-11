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


#ifndef COB_TRICYCLE_CONTROLLER_TRICYCLECTRLTYPES_H
#define COB_TRICYCLE_CONTROLLER_TRICYCLECTRLTYPES_H



struct PlatformState{
    double velX;
    double velY;
    double rotTheta;

    PlatformState() : velX(0.0), velY(0.0), rotTheta(0.0) {}
};

struct WheelState{
    double steer_pos;
    double steer_vel;
    double drive_pos;
    double drive_vel;

    double radius;
    WheelState() : steer_pos(0.0), steer_vel(0.0), drive_pos(0.0), drive_vel(0.0),
                   radius(0.0)
     {}
};

#endif
