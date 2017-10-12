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


#ifndef H_GEOM_CONTROLLER_IMPL
#define H_GEOM_CONTROLLER_IMPL

#include <controller_interface/controller.h>
#include <cob_omni_drive_controller/UndercarriageCtrlGeomROS.h>
#include <boost/scoped_ptr.hpp>

namespace cob_omni_drive_controller
{

template<typename HandleType, typename Controller> class GeomControllerBase {
protected:
    std::vector<HandleType> steer_joints_;
    std::vector<HandleType> drive_joints_;
    std::vector<WheelState> wheel_states_;
    boost::scoped_ptr<Controller> geom_;

public:
    void updateState(){

        for (unsigned i=0; i<wheel_states_.size(); i++){
            if (!geom_->wheels_[i]->geom_.passive){
                wheel_states_[i].dAngGearSteerRad = steer_joints_[i].getPosition();
                wheel_states_[i].dVelGearSteerRadS = steer_joints_[i].getVelocity();
                wheel_states_[i].dVelGearDriveRadS = drive_joints_[i].getVelocity();
            }
            else{
                ///ToDo: for fake wheels state as command
                //ROS_INFO_STREAM("updateState - wheel " << i << ": steer_name " << geom_->wheels_[i]->geom_.steer_name << 
                                            //", dAngGearSteerRad: " << wheel_states_[i].dAngGearSteerRad <<
                                            //", dVelGearSteerRadS: " << wheel_states_[i].dVelGearSteerRadS <<
                                            //", dVelGearDriveRadS: " << wheel_states_[i].dVelGearDriveRadS);
            }
        }
        geom_->updateWheelStates(wheel_states_);
    }
    void updateFakeState(unsigned i, double dAngGearSteerRad, double dVelGearSteerRadS, double dVelGearDriveRadS){
        wheel_states_[i].dAngGearSteerRad = dAngGearSteerRad;
        wheel_states_[i].dVelGearSteerRadS = dVelGearSteerRadS;
        wheel_states_[i].dVelGearDriveRadS = dVelGearDriveRadS;
    }
protected:
    bool setup(const std::vector<typename Controller::WheelParams> &wheel_params){
        if (wheel_params.size() < 3){
            ROS_ERROR("At least three wheel are needed.");
            return false;
        }
        ///check for tricycle mode (1 active + 2 passive wheels)
        unsigned int active = 0;
        unsigned int passive = 0;
        for (unsigned int i=0; i<wheel_params.size(); i++){
            if (wheel_params[i].geom.passive){
                passive++;
                ROS_INFO_STREAM(wheel_params[i].geom.steer_name << " is passive");
            }
            else
            {
                active++;
                ROS_INFO_STREAM(wheel_params[i].geom.steer_name << " is aktive");
            }
        }
        if (passive > 0){
            if (wheel_params.size() == 3 && active == 1 && passive == 2){
                ROS_INFO("Using tricycle_mode");
            }
            else{
                ROS_ERROR("Passive wheels are only allowed in tricycle_mode (1 active + 2 passive)");
                return false;
            }
        }
        wheel_states_.resize(wheel_params.size());
        geom_.reset(new Controller(wheel_params));
        return true;

    }
};

template<typename Interface, typename Controller> class GeomController:
    public GeomControllerBase<typename Interface::ResourceHandleType, Controller>,
        public controller_interface::Controller<Interface> {
public:
    typedef std::vector<typename Controller::WheelParams> wheel_params_t;
    bool init(Interface* hw, ros::NodeHandle& controller_nh){
        std::vector<typename Controller::WheelParams> wheel_params;
        if(!parseWheelParams(wheel_params, controller_nh)) return false;
        return init(hw, wheel_params);
    }
    bool init(Interface* hw, const wheel_params_t & wheel_params){
        if(!this->setup(wheel_params)) return false;
        try{
            for (unsigned i=0; i<wheel_params.size(); i++){
                ROS_INFO_STREAM("wheel_params.size: " << wheel_params.size());
                if (!wheel_params[i].geom.passive){
                    ROS_INFO_STREAM("adding aktive joint " << wheel_params[i].geom.steer_name << ", size: " << this->steer_joints_.size());
                    this->steer_joints_.push_back(hw->getHandle(wheel_params[i].geom.steer_name));
                    this->drive_joints_.push_back(hw->getHandle(wheel_params[i].geom.drive_name));
                    ROS_INFO_STREAM("size: " << this->steer_joints_.size());
                }
                else {
                    //add fake handles for passive wheels
                    ROS_INFO_STREAM("adding fake joint " << wheel_params[i].geom.steer_name << ", size: " << this->steer_joints_.size());
                    this->steer_joints_.resize(this->steer_joints_.size()+1);
                    this->drive_joints_.resize(this->drive_joints_.size()+1);
                    ROS_INFO_STREAM("size: " << this->steer_joints_.size());
                }
            }
        }
        catch(const std::exception &e){
            ROS_ERROR_STREAM("Error while attaching handles: " << e.what());
            return false;
        }
        return true;
    }
};

}

#endif
