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

void UndercarriageGeomBase::WheelData::updateState(const WheelState &state){
    state_ = state;

    // calculate current geometry of robot (exact wheel position, taking into account steering offset of wheels)
    m_dExWheelXPosMM = geom_.dWheelXPosMM + geom_.dDistSteerAxisToDriveWheelMM * sin(state_.dAngGearSteerRad);
    m_dExWheelYPosMM = geom_.dWheelYPosMM - geom_.dDistSteerAxisToDriveWheelMM * cos(state_.dAngGearSteerRad);

    // calculate distance from platform center to wheel center
    m_dExWheelDistMM = sqrt( (m_dExWheelXPosMM * m_dExWheelXPosMM) + (m_dExWheelYPosMM * m_dExWheelYPosMM) );

    // calculate direction of rotational vector
    m_dExWheelAngRad = MathSup::atan4quad( m_dExWheelYPosMM, m_dExWheelXPosMM);

    m_dVelWheelMMS = geom_.dRadiusWheelMM * (state_.dVelGearDriveRadS - dFactorVel* state_.dVelGearSteerRadS);
}

double UndercarriageGeomBase::WheelData::mergeRotRobRadS(const WheelData &wheel1, const WheelData &wheel2){
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

double UndercarriageGeomBase::WheelData::getVelX() const {
    return m_dVelWheelMMS*cos(state_.dAngGearSteerRad);
}
double UndercarriageGeomBase::WheelData::getVelY() const {
    return m_dVelWheelMMS*sin(state_.dAngGearSteerRad);
}
UndercarriageGeom::UndercarriageGeom(const std::vector<WheelParams> &params){
    for(std::vector<WheelParams>::const_iterator it = params.begin(); it != params.end(); ++it){
        wheels_.push_back(WheelData(it->geom));
    }
}

void UndercarriageGeom::calcDirect(PlatformState &state) const{
    UndercarriageGeomBase::calcDirect(state, wheels_);
}

void UndercarriageGeom::updateWheelStates(const std::vector<WheelState> &states){
    UndercarriageGeomBase::updateWheelStates(wheels_, states);
}

void UndercarriageCtrl::CtrlData::setTarget(const PlatformState &plt_state){
    // check if zero movement commanded -> keep orientation of wheels, set wheel velocity to zero
    if((plt_state.dVelLongMMS == 0) && (plt_state.dVelLatMMS == 0) && (plt_state.dRotRobRadS == 0))
    {
        m_dVelGearDriveTargetRadS = 0.0;
        m_dAngGearSteerTargetRad = state_.dAngGearSteerRad;
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
                                        (dtempAxVelYRobMMS * dtempAxVelYRobMMS) ) / geom_.dRadiusWheelMM;
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

double UndercarriageCtrl::limitValue(double value, double limit){
    if(limit != 0){
        if (value > limit){
                value = limit;
        } else if (value < -limit) {
                value = -limit;
        }
    }
    return value;
}

void UndercarriageCtrl::CtrlData::calcControlStep(WheelCommand &command, double dCmdRateS, bool reset){
    if(reset){
        this->reset();
        command.dVelGearDriveRadS = 0.0;
        command.dVelGearSteerRadS = 0.0;
        command.dAngGearSteerRad = state_.dAngGearSteerRad;
        command.dAngGearSteerRadDelta = 0.0;
        return;
    }

    // Normalize Actual Wheel Position before calculation
    double dCurrentPosWheelRAD = angles::normalize_angle(state_.dAngGearSteerRad);
    command.dAngGearSteerRadDelta = angles::normalize_angle(m_dAngGearSteerTargetRad - dCurrentPosWheelRAD);

    // Impedance-Ctrl
    // Calculate resulting desired forces, velocities
    // double dForceDamp, dForceProp, dAccCmd, dVelCmdInt;
    double dForceDamp = - params_.dDamp *m_dCtrlVelCmdInt;
    double dForceProp = params_.dSpring * command.dAngGearSteerRadDelta;

    double dAccCmd = (dForceDamp + dForceProp) /  params_.dVirtM;
    dAccCmd = limitValue(dAccCmd, params_.dDDPhiMax);

    double dVelCmdInt =m_dCtrlVelCmdInt + dCmdRateS * dAccCmd;
    dVelCmdInt = limitValue(dVelCmdInt, params_.dDPhiMax);

    // Store internal ctrlr-states
    m_dCtrlVelCmdInt = dVelCmdInt;

    // set outputs
    command.dVelGearSteerRadS = limitValue(dVelCmdInt, params_.dMaxSteerRateRadpS);
    command.dVelGearDriveRadS = limitValue(m_dVelGearDriveTargetRadS + m_dAngGearSteerTargetRad * dFactorVel, params_.dMaxDriveRateRadpS);

    // provisorial --> skip interpolation and always take Target
    command.dAngGearSteerRad = m_dAngGearSteerTargetRad;
}

void UndercarriageCtrl::CtrlData::reset(){
    //m_dCtrlDeltaPhi = 0.0;
    m_dCtrlVelCmdInt = 0.0;
}

UndercarriageCtrl::UndercarriageCtrl(const std::vector<WheelParams> &params){
    for(std::vector<WheelParams>::const_iterator it = params.begin(); it != params.end(); ++it){
        wheels_.push_back(CtrlData(*it));
    }
}

void UndercarriageCtrl::calcDirect(PlatformState &state) const{
    UndercarriageGeomBase::calcDirect(state, wheels_);
}

void UndercarriageCtrl::updateWheelStates(const std::vector<WheelState> &states){
    UndercarriageGeomBase::updateWheelStates(wheels_, states);
}

void UndercarriageCtrl::setTarget(const PlatformState &state)
{
    for(std::vector<CtrlData>::iterator it = wheels_.begin(); it != wheels_.end(); ++it){
        it->setTarget(state);
    }
}

void UndercarriageCtrl::calcControlStep(std::vector<WheelCommand> &commands, double dCmdRateS, bool reset)
{
    commands.resize(wheels_.size());

    for(size_t i = 0; i < wheels_.size(); ++i){
        wheels_[i].calcControlStep(commands[i], dCmdRateS, reset);
    }

}

void UndercarriageCtrl::reset()
{
    for(size_t i = 0; i < wheels_.size(); ++i){
        wheels_[i].reset();
    }

}
