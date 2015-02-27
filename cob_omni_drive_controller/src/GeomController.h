#ifndef H_GEOM_CONTROLLER_IMPL
#define H_GEOM_CONTROLLER_IMPL

#include <controller_interface/controller.h>
#include <cob_omni_drive_controller/UndercarriageCtrlGeomROS.h>

namespace cob_omni_drive_controller
{

template<typename Interface> class GeomController: public controller_interface::Controller<Interface> {
protected:
    std::vector<typename Interface::ResourceHandleType> steer_joints_, drive_joints_;
    std::vector<UndercarriageCtrlGeom::WheelState> wheel_states_;
    boost::scoped_ptr<UndercarriageCtrlGeom> geom_;
public:    
    bool init(Interface* hw, ros::NodeHandle& controller_nh){

        std::vector<UndercarriageCtrlGeom::WheelParams> wheel_params;
        if(!parseWheelParams(wheel_params, controller_nh)) return false;

        if (wheel_params.size() < 3){
            ROS_ERROR("At least three wheel are needed.");
            return false;
        }
        try{
            for (unsigned i=0; i<wheel_params.size(); i++){
                steer_joints_.push_back(hw->getHandle(wheel_params[i].steer_name));
                drive_joints_.push_back(hw->getHandle(wheel_params[i].drive_name));
            }
        }
        catch(const std::exception &e){
            ROS_ERROR_STREAM("Error while attaching handles: " << e.what());
            return false;
        }

        wheel_states_.resize(wheel_params.size());
        geom_.reset(new UndercarriageCtrlGeom(wheel_params));
        return true;
    }

    void update(){

        for (unsigned i=0; i<wheel_states_.size(); i++){
            wheel_states_[i].dAngGearSteerRad = steer_joints_[i].getPosition();
            wheel_states_[i].dVelGearSteerRadS = steer_joints_[i].getVelocity();
            wheel_states_[i].dVelGearDriveRadS = drive_joints_[i].getVelocity();
        }
        geom_->updateWheelStates(wheel_states_);
    }    
};


}

#endif
