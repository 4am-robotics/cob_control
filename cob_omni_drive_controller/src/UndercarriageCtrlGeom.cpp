/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_undercarriage_ctrl
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: April 2010:
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <cob_omni_drive_controller/UndercarriageCtrlGeom.h>

#include <math.h>
#include <angles/angles.h>
#include <stdexcept>

namespace MathSup {
    const double PI = 3.14159265358979323846;
    void normalizePi(double &val) { val = angles::normalize_angle(val); }

    double atan4quad(double y, double x)
    {
        if((x==0.0) && (y==0.0)) return 0; // special case (?)
        return atan2(y,x);
    }

}
double getWeightedDelta(double current_position, double old_target, double new_target){
    // current_position =  angles::normalize_angle(current_position); not needed if current Pos is alread normalized

    // Calculate differences between current config to possible set-points
    double dtempDeltaPhi1RAD = angles::normalize_angle( new_target - current_position );
    double dtempDeltaPhiCmd1RAD = angles::normalize_angle(new_target - old_target);

    // determine optimal setpoint value
    // 1st which set point is closest to current cinfog
    //     but: avoid permanent switching (if next target is about PI/2 from current config)
    // 2nd which set point is closest to last set point
    // "fitness criteria" to choose optimal set point:
    // calculate accumulted (+ weighted) difference between targets, current config. and last command
    return 0.6*fabs(dtempDeltaPhi1RAD) + 0.4*fabs(dtempDeltaPhiCmd1RAD);
}

void UndercarriageCtrlGeom::WheelData::setTarget(const PlatformState &plt_state){
    // check if zero movement commanded -> keep orientation of wheels, set wheel velocity to zero
    if((plt_state.dVelLongMMS == 0) && (plt_state.dVelLatMMS == 0) && (plt_state.dRotRobRadS == 0))
    {
        m_dVelGearDriveTargetRadS = state_.dAngGearSteerRad;
        m_dAngGearSteerTargetRad = 0.0;
        return;
    }

    // calculate velocity and direction of single wheel motion
    // Translational Portion
    double dtempAxVelXRobMMS = plt_state.dVelLongMMS;
    double dtempAxVelYRobMMS = plt_state.dVelLatMMS;
    // Rotational Portion
    dtempAxVelXRobMMS += plt_state.dRotRobRadS * m_dExWheelDistMM * -sin(m_dExWheelAngRad);
    dtempAxVelYRobMMS += plt_state.dRotRobRadS * m_dExWheelDistMM * cos(m_dExWheelAngRad);

    // calculate resulting steering angle
    // Wheel has to move in direction of resulting velocity vector of steering axis
    double dAngGearSteerTarget1Rad = MathSup::atan4quad(dtempAxVelYRobMMS, dtempAxVelXRobMMS);
    // calculate corresponding angle in opposite direction (+180 degree)
    double dAngGearSteerTarget2Rad = dAngGearSteerTarget1Rad + MathSup::PI;
    MathSup::normalizePi(dAngGearSteerTarget2Rad);

    // calculate absolute value of rotational rate of driving wheels in rad/s
    double dVelGearDriveTarget1RadS = sqrt( (dtempAxVelXRobMMS * dtempAxVelXRobMMS) +
                                        (dtempAxVelYRobMMS * dtempAxVelYRobMMS) ) / params_.dRadiusWheelMM;
    // now adapt to direction (forward/backward) of wheel
    double dVelGearDriveTarget2RadS = -dVelGearDriveTarget1RadS;


    if(getWeightedDelta(state_.dAngGearSteerRad,  m_dAngGearSteerTargetRad, dAngGearSteerTarget1Rad)
        <= getWeightedDelta(state_.dAngGearSteerRad, m_dAngGearSteerTargetRad, dAngGearSteerTarget2Rad))
    {
        // Target1 is "optimal"
        m_dVelGearDriveTargetRadS = dVelGearDriveTarget1RadS;
        m_dAngGearSteerTargetRad = dAngGearSteerTarget1Rad;
    }
    else
    {
        // Target2 is "optimal"
        m_dVelGearDriveTargetRadS = dVelGearDriveTarget2RadS;
        m_dAngGearSteerTargetRad = dAngGearSteerTarget2Rad;
    }

}

void UndercarriageCtrlGeom::WheelData::updateState(const WheelState &state){
    state_ = state;

    // calculate current geometry of robot (exact wheel position, taking into account steering offset of wheels)
    m_dExWheelXPosMM = params_.dWheelXPosMM + params_.dDistSteerAxisToDriveWheelMM * sin(state_.dAngGearSteerRad);
    m_dExWheelYPosMM = params_.dWheelYPosMM - params_.dDistSteerAxisToDriveWheelMM * cos(state_.dAngGearSteerRad);

    // calculate distance from platform center to wheel center
    m_dExWheelDistMM = sqrt( (m_dExWheelXPosMM * m_dExWheelXPosMM) + (m_dExWheelYPosMM * m_dExWheelYPosMM) );

    // calculate direction of rotational vector
    m_dExWheelAngRad = MathSup::atan4quad( m_dExWheelYPosMM, m_dExWheelXPosMM);

    m_dVelWheelMMS = params_.dRadiusWheelMM * (state_.dVelGearDriveRadS - params_.dFactorVel* state_.dVelGearSteerRadS);
}

double UndercarriageCtrlGeom::WheelData::mergeRotRobRadS(const WheelData &wheel1, const WheelData &wheel2){
    // calc Parameters (Dist,Phi) of virtual linking axis of the two considered wheels
    double dtempDiffXMM = wheel2.m_dExWheelXPosMM - wheel1.m_dExWheelXPosMM;
    double dtempDiffYMM = wheel2.m_dExWheelYPosMM - wheel1.m_dExWheelYPosMM;

    double dtempRelDistWheelsMM = sqrt( dtempDiffXMM*dtempDiffXMM + dtempDiffYMM*dtempDiffYMM );
    double dtempRelPhiWheelsRAD = MathSup::atan4quad( dtempDiffYMM, dtempDiffXMM );

    // transform velocity of wheels into relative coordinate frame of linking axes -> subtract angles
    double dtempRelPhiWheel1RAD = wheel1.state_.dAngGearSteerRad - dtempRelPhiWheelsRAD;
    double dtempRelPhiWheel2RAD = wheel2.state_.dAngGearSteerRad - dtempRelPhiWheelsRAD;

    return (wheel2.m_dVelWheelMMS * sin(dtempRelPhiWheel2RAD) - wheel1.m_dVelWheelMMS * sin(dtempRelPhiWheel1RAD))/dtempRelDistWheelsMM;
}
double limit_value(double value, double limit){
    if (value > limit){
            value = limit;
    } else if (value < -limit) {
            value = -limit;
    }
    return value;
}
void UndercarriageCtrlGeom::WheelData::reset(){
    //m_dCtrlDeltaPhi = 0.0;
    m_dCtrlVelCmdInt = 0.0;

}
void UndercarriageCtrlGeom::WheelData::calcControlStep(WheelState &command, double dCmdRateS, bool reset){
    if(reset){
        this->reset();
        command.dVelGearDriveRadS = 0.0;
        command.dVelGearSteerRadS = 0.0;
        command.dAngGearSteerRad = state_.dAngGearSteerRad;
        return;
    }

    // Normalize Actual Wheel Position before calculation
    double dCurrentPosWheelRAD = angles::normalize_angle(state_.dAngGearSteerRad);
    double dDeltaPhi = angles::normalize_angle(m_dAngGearSteerTargetRad - dCurrentPosWheelRAD);

    // Impedance-Ctrl
    // Calculate resulting desired forces, velocities
    // double dForceDamp, dForceProp, dAccCmd, dVelCmdInt;
    double dForceDamp = - params_.dDamp *m_dCtrlVelCmdInt;
    double dForceProp = params_.dSpring * dDeltaPhi;

    double dAccCmd = (dForceDamp + dForceProp) /  params_.dVirtM;
    dAccCmd = limit_value(dAccCmd, params_.dDDPhiMax);

    double dVelCmdInt =m_dCtrlVelCmdInt + dCmdRateS * dAccCmd;
    dVelCmdInt = limit_value(dVelCmdInt, params_.dDPhiMax);

    // Store internal ctrlr-states
    m_dCtrlVelCmdInt = dVelCmdInt;

    // set outputs
    command.dVelGearSteerRadS = limit_value(dVelCmdInt, params_.dMaxSteerRateRadpS);

    command.dVelGearDriveRadS = m_dVelGearDriveTargetRadS + m_dAngGearSteerTargetRad * params_.dFactorVel;

    // provisorial --> skip interpolation and always take Target
    command.dAngGearSteerRad = m_dAngGearSteerTargetRad;
}

// Constructor
UndercarriageCtrlGeom::UndercarriageCtrlGeom(const std::vector<WheelParams> &params){
    for(std::vector<WheelParams>::const_iterator it = params.begin(); it != params.end(); ++it){
        wheels_.push_back(WheelData(*it));
    }

//  // init Prms of Impedance-Ctrlr
//  m_dSpring = 10.0;
//  m_dDamp = 2.5;
//  m_dVirtM = 0.1;
//  m_dDPhiMax = 12.0;
//  m_dDDPhiMax = 100.0;
}

// Set desired value for Plattfrom Velocity to UndercarriageCtrl (Sollwertvorgabe)
void UndercarriageCtrlGeom::setTarget(const PlatformState &state)
{
    for(std::vector<WheelData>::iterator it = wheels_.begin(); it != wheels_.end(); ++it){
        it->setTarget(state);
    }
}

// Set actual values of wheels (steer/drive velocity/position) (Istwerte)
void UndercarriageCtrlGeom::updateWheelStates(const std::vector<WheelState> &states)
{
    if(wheels_.size() != states.size()) throw std::length_error("number of states does not match number of wheels");

    for(size_t i = 0; i < wheels_.size(); ++i){
        wheels_[i].updateState(states[i]);
    }
}

// calculate direct kinematics
void UndercarriageCtrlGeom::calcDirect(PlatformState &state)
{
    double dtempRotRobRADPS = 0;    // Robot-Rotation-Rate in rad/s (in Robot-Coordinateframe)
    double dtempVelXRobMMS = 0;     // Robot-Velocity in x-Direction (longitudinal) in mm/s (in Robot-Coordinateframe)
    double dtempVelYRobMMS = 0;     // Robot-Velocity in y-Direction (lateral) in mm/s (in Robot-Coordinateframe)

    // calculate rotational rate of robot and current "virtual" axis between all wheels
    for(int i = 0; i < wheels_.size() ; i++)
    {
        WheelData &wheel = wheels_[i];
        WheelData &other_wheel = wheels_[(i+1) % wheels_.size()];

        dtempRotRobRADPS += WheelData::mergeRotRobRadS(wheel, other_wheel);
        dtempVelXRobMMS += wheel.m_dVelWheelMMS*cos(wheel.state_.dAngGearSteerRad);
        dtempVelYRobMMS += wheel.m_dVelWheelMMS*sin(wheel.state_.dAngGearSteerRad);
    }

    // assign rotational velocities for output
    state.dRotRobRadS = dtempVelXRobMMS/wheels_.size();

    // assign linear velocity of robot for output
    state.dVelLongMMS = dtempVelXRobMMS/wheels_.size();
    state.dVelLatMMS = dtempVelYRobMMS/wheels_.size();

}

// perform one discrete Control Step (controls steering angle)
void UndercarriageCtrlGeom::calcControlStep(std::vector<WheelState> &states, double dCmdRateS, bool reset)
{
    if(wheels_.size() != states.size()) throw std::length_error("number of states does not match number of wheels");

    for(size_t i = 0; i < wheels_.size(); ++i){
        wheels_[i].calcControlStep(states[i], dCmdRateS, reset);
    }

}

void UndercarriageCtrlGeom::reset()
{
    for(size_t i = 0; i < wheels_.size(); ++i){
        wheels_[i].reset();
    }

}

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp>

void UndercarriageCtrlGeom::parseIniFiles(std::vector<WheelParams> &params, const std::string &path){
    params.clear();

    using boost::property_tree::ptree;
    ptree pt;

    read_ini(path + "Platform.ini", pt);
    read_ini(path + "MotionCtrl.ini", pt);
    
    WheelParams param;

    // Prms of Impedance-Ctrlr
    param.dSpring = pt.get("SteerCtrl.Spring", 10.0);
    param.dDamp = pt.get("SteerCtrl.Damp", 2.5);
    param.dVirtM = pt.get("SteerCtrl.VirtMass", 0.1);
    param.dDPhiMax = pt.get("SteerCtrl.DPhiMax", 12.0);
    param.dDDPhiMax= pt.get("SteerCtrl.DDPhiMax", 100.0);


    param.dRadiusWheelMM = pt.get<double>("Geom.RadiusWheel");
    param.dDistSteerAxisToDriveWheelMM = pt.get<double>("Geom.DistSteerAxisToDriveWheelCenter", 0.0);

    param.dMaxDriveRateRadpS = pt.get<double>("DrivePrms.MaxDriveRate");
    param.dMaxSteerRateRadpS = pt.get<double>("DrivePrms.dMaxSteerRateRadpS");


    int num_wheels = pt.get<int>("Config.NumberOfWheels");
    for(int i=1; i <= num_wheels; ++i){
        std::string num = boost::lexical_cast<std::string>(i);
        param.dWheelXPosMM = pt.get<double>("Geom.Wheel"+num+"XPos");
        param.dWheelYPosMM = pt.get<double>("Geom.Wheel"+num+"YPos");

        double deg = pt.get<double>("DrivePrms.Wheel"+num+"NeutralPosition", 0.0);
        param.dWheelNeutralPos = angles::from_degrees(deg);

        double coupling = pt.get<double>("DrivePrms.Wheel"+num+"SteerDriveCoupling", 0.0);

        param.dFactorVel = - coupling + param.dDistSteerAxisToDriveWheelMM / param.dRadiusWheelMM;

        params.push_back(param);
    }

}
