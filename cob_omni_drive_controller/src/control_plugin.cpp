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


#include "GeomController.h"
#include "WheelControllerBase.h"

#include <dynamic_reconfigure/server.h>
#include <cob_omni_drive_controller/SteerCtrlConfig.h>

namespace cob_omni_drive_controller
{

class WheelController : public WheelControllerBase< GeomController<hardware_interface::VelocityJointInterface, UndercarriageCtrl> >
{
public:
    virtual bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh){

        wheel_params_t wheel_params;
        if(!parseWheelParams(wheel_params, controller_nh) || !GeomController::init(hw, wheel_params)) return false;

        pos_ctrl_.init(wheel_params, controller_nh);

        return setup(root_nh, controller_nh);
    }
    virtual void update(const ros::Time& time, const ros::Duration& period){

        updateState();
        pos_ctrl_.try_configure(*geom_);

        updateCtrl(time, period);

        for (unsigned i=0; i<wheel_commands_.size(); i++){
            steer_joints_[i].setCommand(wheel_commands_[i].dVelGearSteerRadS);
            drive_joints_[i].setCommand(wheel_commands_[i].dVelGearDriveRadS);
        }

    }
    class PosCtrl {
    public:
        PosCtrl() : updated(false) {}
        void try_configure(UndercarriageCtrl &ctrl){
            boost::recursive_mutex::scoped_try_lock lock(mutex);
            if(lock && updated){
                ctrl.configure(pos_ctrl_params);
                updated = false;
            }
        }
        void init(const wheel_params_t &params, const ros::NodeHandle &nh){
            boost::recursive_mutex::scoped_lock lock(mutex);
            pos_ctrl_params.resize(params.size());
            reconfigure_server_axes_.clear();

            reconfigure_server_.reset( new dynamic_reconfigure::Server<SteerCtrlConfig>(mutex,ros::NodeHandle(nh, "default/steer_ctrl")));
            reconfigure_server_->setCallback(boost::bind(&PosCtrl::setForAll, this, _1, _2)); // this writes the default into pos_ctrl_params
            {
                SteerCtrlConfig config;
                copy(config, params.front().pos_ctrl);
                reconfigure_server_->setConfigDefault(config);
            }

            for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                boost::shared_ptr<dynamic_reconfigure::Server<SteerCtrlConfig> > dr(new dynamic_reconfigure::Server<SteerCtrlConfig>(mutex, ros::NodeHandle(nh, params[i].geom.steer_name)));
                SteerCtrlConfig config;
                copy(config, params[i].pos_ctrl);
                dr->setConfigDefault(config);
                dr->updateConfig(config);
                dr->setCallback(boost::bind(&PosCtrl::setForOne, this, i, _1, _2));  // this writes the joint-specific config into pos_ctrl_params
                reconfigure_server_axes_.push_back(dr);
            }
        }
    private:
        static void copy(PosCtrlParams &params, const SteerCtrlConfig &config){
            params.dSpring = config.spring;
            params.dDamp = config.damp;
            params.dVirtM = config.virt_mass;
            params.dDPhiMax = config.d_phi_max;
            params.dDDPhiMax = config.dd_phi_max;
        }
        static void copy(SteerCtrlConfig &config, const PosCtrlParams &params){
            config.spring = params.dSpring;
            config.damp = params.dDamp;
            config.virt_mass = params.dVirtM;
            config.d_phi_max = params.dDPhiMax;
            config.dd_phi_max = params.dDDPhiMax;
        }
        // these function don't get locked
        void setForAll(SteerCtrlConfig &config, uint32_t /*level*/) {
            ROS_INFO("configure all steers: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
            for(size_t i=0; i< pos_ctrl_params.size(); ++i) {
                copy(pos_ctrl_params[i], config);
                if(!reconfigure_server_axes_.empty()){
                    reconfigure_server_axes_[i]->setConfigDefault(config);
                    reconfigure_server_axes_[i]->updateConfig(config);
                }
            }
            updated = true;
        }
        void setForOne(size_t i, SteerCtrlConfig &config, uint32_t /*level*/) {
            ROS_INFO("configure steer %d: s: %lf, d: %lf, m: %lf, v: %lf, a: %lf", (int)i, config.spring, config.damp, config.virt_mass, config.d_phi_max, config.dd_phi_max);
            copy(pos_ctrl_params[i], config);
            updated = true;
        }
        std::vector<PosCtrlParams> pos_ctrl_params;
        boost::recursive_mutex mutex; // dynamic_reconfigure::Server calls the callback from the setCallback function
        bool updated;
        boost::scoped_ptr< dynamic_reconfigure::Server<SteerCtrlConfig> > reconfigure_server_;
        std::vector<boost::shared_ptr< dynamic_reconfigure::Server<SteerCtrlConfig> > > reconfigure_server_axes_; // TODO: use unique_ptr
    } pos_ctrl_;

};

}

PLUGINLIB_EXPORT_CLASS( cob_omni_drive_controller::WheelController, controller_interface::ControllerBase)
