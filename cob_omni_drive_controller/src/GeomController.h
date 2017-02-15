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
    std::vector<typename Controller::WheelState> wheel_states_;
    boost::scoped_ptr<Controller> geom_;

public:
    void updateState(){

        for (unsigned i=0; i<wheel_states_.size(); i++){
            wheel_states_[i].dAngGearSteerRad = steer_joints_[i].getPosition();
            wheel_states_[i].dVelGearSteerRadS = steer_joints_[i].getVelocity();
            wheel_states_[i].dVelGearDriveRadS = drive_joints_[i].getVelocity();
        }
        geom_->updateWheelStates(wheel_states_);
    }
protected:
    bool setup(const std::vector<typename Controller::WheelParams> &wheel_params){
        if (wheel_params.size() < 3){
            ROS_ERROR("At least three wheel are needed.");
            return false;
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
    bool init(Interface* hw, ros::NodeHandle& controller_nh){
        std::vector<typename Controller::WheelParams> wheel_params;
        if(!parseWheelParams(wheel_params, controller_nh)) return false;
        if(!this->setup(wheel_params)) return false;
        try{
            for (unsigned i=0; i<wheel_params.size(); i++){
                this->steer_joints_.push_back(hw->getHandle(wheel_params[i].geom.steer_name));
                this->drive_joints_.push_back(hw->getHandle(wheel_params[i].geom.drive_name));
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
