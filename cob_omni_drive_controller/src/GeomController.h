#ifndef H_GEOM_CONTROLLER_IMPL
#define H_GEOM_CONTROLLER_IMPL

#include <controller_interface/controller.h>
#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>

namespace cob_omni_drive_controller
{

template<typename Interface> class GeomController: public controller_interface::Controller<Interface> {
protected:
    std::vector<typename Interface::ResourceHandleType> steer_joints_, drive_joints_;
    std::vector<UndercarriageCtrlGeom::WheelState> wheel_states_;
    boost::scoped_ptr<UndercarriageCtrlGeom> geom_;
public:    
    bool init(Interface* hw, ros::NodeHandle& controller_nh){

        std::vector<std::string> steer_names, drive_names;

        if (!controller_nh.getParam("steer_joints", steer_names)){
            ROS_ERROR("Parameter 'steer_joints' not set");
            return false;
        }
        if (!controller_nh.getParam("drives_joints", drive_names)){
            ROS_ERROR("Parameter 'drives_joints' not set");
            return false;
        }

        if (steer_names.size()!=drive_names.size()){
            ROS_ERROR("Number of steer joints does not match number of drive joints");
            return false;
        }

        if (drive_names.size() < 3){
            ROS_ERROR("At least three wheel are needed.");
            return false;
        }

        for (unsigned i=0; i<steer_names.size(); i++){
            steer_joints_.push_back(hw->getHandle(steer_names[i]));
            drive_joints_.push_back(hw->getHandle(drive_names[i]));
        }
        wheel_states_.resize(steer_names.size());

        std::vector<UndercarriageCtrlGeom::WheelParams> params;

        std::string ini_directory;
        if (!controller_nh.getParam("ini_directory", ini_directory)){
            ROS_ERROR("Parameter 'ini_directory' not set");
            return false;
        }

        try{
            UndercarriageCtrlGeom::parseIniFiles(params, ini_directory);
        }
        catch(...){
            ROS_ERROR("INI file parsing failed");
            return false;
        }
        geom_.reset(new UndercarriageCtrlGeom(params));
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