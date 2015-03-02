#include <cob_omni_drive_controller/UndercarriageCtrlGeomROS.h>
#include <urdf/model.h>
#include <angles/angles.h>

class MergedXmlRpcStruct : public XmlRpc::XmlRpcValue{
    MergedXmlRpcStruct(const XmlRpc::XmlRpcValue& a) :XmlRpc::XmlRpcValue(a){ assertStruct(); }
public:
    MergedXmlRpcStruct(){ assertStruct(); }
    MergedXmlRpcStruct(const XmlRpc::XmlRpcValue& a, const MergedXmlRpcStruct &b, bool recursive= true) :XmlRpc::XmlRpcValue(a){
        assertStruct();

        for(ValueStruct::const_iterator it = b._value.asStruct->begin(); it != b._value.asStruct->end(); ++it){
            std::pair<XmlRpc::XmlRpcValue::iterator,bool> res =  _value.asStruct->insert(*it);

            if(recursive && !res.second && res.first->second.getType() == XmlRpc::XmlRpcValue::TypeStruct && it->second.getType() == XmlRpc::XmlRpcValue::TypeStruct){
                res.first->second = MergedXmlRpcStruct(res.first->second, it->second); // recursive struct merge with implicit cast
            }
        }


    }
};

template<typename T> bool try_read(T& val, const std::string &name, XmlRpc::XmlRpcValue &wheel, const T& def=T()){
    if(wheel.hasMember(name)){
        val = (T)wheel[name];
        return true;
    }else{
        val = def;
        return false;
    }
}

template<typename T> bool read(T& val, const std::string &name, XmlRpc::XmlRpcValue &wheel){
    if(wheel.hasMember(name)){
        val = wheel[name];
        return true;
    }else{
        ROS_ERROR_STREAM("Parameter not found: " << name);
        return false;
    }
}

bool parseCtrlParams(UndercarriageCtrl::CtrlParams & params, XmlRpc::XmlRpcValue &wheel){
    double deg;
    try_read(deg, "steer_neutral_position", wheel);
    params.dWheelNeutralPos = angles::from_degrees(deg);

    try_read(params.dMaxSteerRateRadpS, "max_steer_rate", wheel);
    try_read(params.dMaxDriveRateRadpS, "max_drive_rate", wheel);

    if(!wheel.hasMember("steer_ctrl")){
        ROS_ERROR_STREAM("steer_ctrl not found");
        return false;
    }
    XmlRpc::XmlRpcValue &steer = wheel["steer_ctrl"];

    return read(params.dSpring, "spring", steer)
        && read(params.dDamp, "damp", steer)
        && read(params.dVirtM, "virt_mass", steer)
        && read(params.dDPhiMax, "d_phi_max", steer)
        && read(params.dDDPhiMax, "dd_phi_max", steer);
}

bool parseWheelGeom(UndercarriageGeom::WheelGeom & geom, XmlRpc::XmlRpcValue &wheel, MergedXmlRpcStruct &merged, urdf::Model* model){

    try_read(geom.steer_name, "steer", wheel);
    try_read(geom.drive_name, "drive", wheel);
    try_read(geom.dSteerDriveCoupling, "steer_drive_coupling", wheel);

    boost::shared_ptr<const urdf::Joint> steer_joint;
    urdf::Vector3 steer_pos;

    if(model && !geom.steer_name.empty()){
        steer_joint = model->getJoint(geom.steer_name);
        if(steer_joint){
            steer_pos = steer_joint->parent_to_joint_origin_transform.position;
        }
    }

    if(!try_read(steer_pos.x, "x_pos", wheel) && !steer_joint){
        ROS_ERROR_STREAM("Could not parse x_pos");
        return false;
    }

    if(!try_read(steer_pos.y, "y_pos", wheel) && !steer_joint){
        ROS_ERROR_STREAM("Could not parse y_pos");
        return false;
    }

    if(!try_read(steer_pos.z, "wheel_radius", merged) && !steer_joint){
        ROS_ERROR_STREAM("Could not parse wheel_radius");
        return false;
    }

    geom.dWheelXPosMM = steer_pos.x * 1000;
    geom.dWheelYPosMM = steer_pos.y * 1000;
    geom.dRadiusWheelMM = steer_pos.z * 1000;

    double offset;

    if(!try_read(offset, "wheel_offset", merged)){
        boost::shared_ptr<const urdf::Joint> drive_joint;
        if(model && !geom.drive_name.empty()){
            drive_joint = model->getJoint(geom.drive_name);
        }
        if(drive_joint){
            const urdf::Vector3& pos = drive_joint->parent_to_joint_origin_transform.position;
            offset = sqrt(pos.x*pos.x + pos.y * pos.y);
        }else{
            ROS_ERROR_STREAM("Could not parse wheel_offset");
            return false;
        }
    }
    geom.dDistSteerAxisToDriveWheelMM = offset * 1000;
    return true;
}

template<typename W> bool parseWheel(W & params, XmlRpc::XmlRpcValue &wheel, MergedXmlRpcStruct &merged, urdf::Model* model);

template<> bool parseWheel(UndercarriageGeom::WheelParams & params, XmlRpc::XmlRpcValue &wheel, MergedXmlRpcStruct &merged, urdf::Model* model){
    return parseWheelGeom(params.geom, wheel, merged, model);
}

template<> bool parseWheel(UndercarriageCtrl::WheelParams & params, XmlRpc::XmlRpcValue &wheel, MergedXmlRpcStruct &merged, urdf::Model* model){
    return parseWheelGeom(params.geom, wheel, merged, model) && parseCtrlParams(params.ctrl, merged);
}

template<typename W> bool parseWheels(std::vector<W> &wheel_params, const ros::NodeHandle &nh, bool read_urdf){

    urdf::Model model;

    std::string description_name;
    bool has_model = read_urdf && nh.searchParam("robot_description", description_name) &&  model.initParam(description_name);

    MergedXmlRpcStruct defaults;
    nh.getParam("defaults", defaults);

    // clear vector in case of reinititialization
    wheel_params.clear();

    XmlRpc::XmlRpcValue wheel_list;
    if (!nh.getParam("wheels", wheel_list)){
        ROS_ERROR("List of wheels not found");
        return false;
    }

    for(int i = 0; i < wheel_list.size(); ++i){

        W params;
        XmlRpc::XmlRpcValue & wheel = wheel_list[i];
        MergedXmlRpcStruct merged(wheel, defaults);

        if(!parseWheel(params, wheel, merged, has_model?&model:0)){
            return false;
        }

        wheel_params.push_back(params);
    }
    return !wheel_params.empty();
}

namespace cob_omni_drive_controller{

bool parseWheelParams(std::vector<UndercarriageGeom::WheelParams> &params, const ros::NodeHandle &nh, bool read_urdf /* = true*/){
    return parseWheels(params, nh, read_urdf);
}
bool parseWheelParams(std::vector<UndercarriageCtrl::WheelParams> &params, const ros::NodeHandle &nh, bool read_urdf /* = true*/){
    return parseWheels(params, nh, read_urdf);
}

    
}
