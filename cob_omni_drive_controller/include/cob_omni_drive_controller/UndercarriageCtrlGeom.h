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

#ifndef UndercarriageCtrlGeom_INCLUDEDEF_H
#define UndercarriageCtrlGeom_INCLUDEDEF_H

#include <vector>
#include <string>
#include <stdexcept>
#include <boost/make_shared.hpp>

struct PlatformState{
    double dVelLongMMS;
    double dVelLatMMS;
    double dRotRobRadS;

    const double getVelX() { return dVelLongMMS / 1000.0; }
    const double getVelY() { return dVelLatMMS / 1000.0; }

    void setVelX(const double val) { dVelLongMMS  = val * 1000.0; }
    void setVelY(const double val) { dVelLatMMS = val * 1000.0; }

    PlatformState() : dVelLongMMS(0), dVelLatMMS(0),dRotRobRadS(0) {}
};
struct WheelState {
    double dVelGearDriveRadS;
    double dVelGearSteerRadS;
    double dAngGearSteerRad;
    WheelState() : dVelGearDriveRadS(0), dVelGearSteerRadS(0), dAngGearSteerRad(0) {}
};
struct WheelCommand : public WheelState {
    double dAngGearSteerRadDelta;
    WheelCommand() : dAngGearSteerRadDelta(0) {}
};
struct WheelGeom{
    std::string steer_name, drive_name;

    /** Position of the Wheels' Steering Axis'
    *  in cartesian (X/Y) coordinates
    *  relative to robot coordinate System
    */
    double dWheelXPosMM;
    double dWheelYPosMM;

    double dSteerDriveCoupling;

    double dRadiusWheelMM;
    double dDistSteerAxisToDriveWheelMM;

};
struct WheelData {
    WheelGeom geom_;

    /** Factor between steering motion and steering induced motion of drive wheels
        *  subtract from Drive-Wheel Vel to get effective Drive Velocity (Direct Kinematics)
        *  add to Drive-Wheel Vel (Inverse Kinematics) to account for coupling when commanding velos
        */
    double dFactorVel;

    WheelState state_;

    /** Exact Position of the Wheels' itself
        *  in cartesian (X/Y) and polar (Dist/Ang) coordinates
        *  relative to robot coordinate System
        */
    double m_dExWheelXPosMM;
    double m_dExWheelYPosMM;
    double m_dExWheelDistMM;
    double m_dExWheelAngRad;

    double m_dVelWheelMMS;

    void updateState(const WheelState &state);
    double getVelX() const;
    double getVelY() const;

    static double mergeRotRobRadS(const WheelData &wheel1, const WheelData &wheel2);

    WheelData(const WheelGeom &geom)
    : geom_(geom),
        dFactorVel(-geom_.dSteerDriveCoupling + geom_.dDistSteerAxisToDriveWheelMM / geom_.dRadiusWheelMM) {
        updateState(WheelState());
    }
};

class UndercarriageGeomBase
{
public:
    // Get result of direct kinematics
    virtual void calcDirect(PlatformState &state) const = 0;

    // Set actual values of wheels (steer/drive velocity/position) (Istwerte)
    virtual void updateWheelStates(const std::vector<WheelState> &states) = 0;

    virtual ~UndercarriageGeomBase() {}

protected:
    template<typename V> static void updateWheelStates(V& wheels, const std::vector<WheelState> &states)
    {
        if(wheels.size() != states.size()) throw std::length_error("number of states does not match number of wheels");

        for(size_t i = 0; i < wheels.size(); ++i){
            wheels[i]->updateState(states[i]);
        }
    }

    template<typename V> static void calcDirect(PlatformState &state, const V& wheels)
    {
        double dtempRotRobRADPS = 0;    // Robot-Rotation-Rate in rad/s (in Robot-Coordinateframe)
        double dtempVelXRobMMS = 0;     // Robot-Velocity in x-Direction (longitudinal) in mm/s (in Robot-Coordinateframe)
        double dtempVelYRobMMS = 0;     // Robot-Velocity in y-Direction (lateral) in mm/s (in Robot-Coordinateframe)

        // calculate rotational rate of robot and current "virtual" axis between all wheels
        for(int i = 0; i < wheels.size() ; i++)
        {
            const WheelData &wheel = *wheels[i];
            const WheelData &other_wheel = *wheels[(i+1) % wheels.size()];

            dtempRotRobRADPS += WheelData::mergeRotRobRadS(wheel, other_wheel);
            dtempVelXRobMMS += wheel.getVelX();
            dtempVelYRobMMS += wheel.getVelY();
        }

        // assign rotational velocities for output
        state.dRotRobRadS = dtempRotRobRADPS/wheels.size();

        // assign linear velocity of robot for output
        state.dVelLongMMS = dtempVelXRobMMS/wheels.size();
        state.dVelLatMMS = dtempVelYRobMMS/wheels.size();

    }

};

class UndercarriageGeom : public UndercarriageGeomBase {
public:
    struct WheelParams {
        WheelGeom geom;
    };

    // Constructor
    UndercarriageGeom(const std::vector<WheelParams> &params);

    // Get result of direct kinematics
    virtual void calcDirect(PlatformState &state) const;

    // Set actual values of wheels (steer/drive velocity/position) (Istwerte)
    virtual void updateWheelStates(const std::vector<WheelState> &states);

private:
    std::vector<boost::shared_ptr<WheelData> > wheels_;
};

double limitValue(double value, double limit);

template<typename T> class UndercarriageCtrlBase : public UndercarriageGeomBase {
public:
    // Constructor
    template<typename T2> UndercarriageCtrlBase(const std::vector<T2> &params){
        for(typename std::vector<T2>::const_iterator it = params.begin(); it != params.end(); ++it){
            wheels_.push_back(boost::make_shared<T>(*it));
        }
    }

    // Get result of direct kinematics
    virtual void calcDirect(PlatformState &state) const {
        UndercarriageGeomBase::calcDirect(state, wheels_);
    }

    // Set actual values of wheels (steer/drive velocity/position) (Istwerte)
    virtual void updateWheelStates(const std::vector<WheelState> &states) {
        UndercarriageGeomBase::updateWheelStates(wheels_, states);
    }
    // Set desired value for Plattform Velocity to UndercarriageCtrl (Sollwertvorgabe)
    void setTarget(const PlatformState &state)  {
        for(size_t i = 0; i < wheels_.size(); ++i){
            wheels_[i]->setTarget(state);
        }
    }

    // Get set point values for the Wheels (including controller) from UndercarriangeCtrl
    void calcControlStep(std::vector<WheelCommand> &commands, double dCmdRateS, bool reset) {
        commands.resize(wheels_.size());

        for(size_t i = 0; i < wheels_.size(); ++i){
            wheels_[i]->calcControlStep(commands[i], dCmdRateS, reset);
        }
    }

    void reset() {
        for(size_t i = 0; i < wheels_.size(); ++i){
            wheels_[i]->reset();
        }
    }

protected:
    std::vector<boost::shared_ptr<T> > wheels_;
};

struct CtrlParams{
    double dWheelNeutralPos;

    double dMaxDriveRateRadpS;
    double dMaxSteerRateRadpS;
};

struct WheelCtrlParams {
    WheelGeom geom;
    CtrlParams ctrl;
};

struct CtrlData : public WheelData {
    CtrlParams params_;

    double m_dAngGearSteerTargetRad; // choosen alternativ for steering angle
    double m_dVelGearDriveTargetRadS;

    // calculate inverse kinematics
    void setTarget(const PlatformState &state);

    virtual void calcControlStep(WheelCommand &command, double dCmdRateS, bool reset);

    virtual void reset();

    template<typename P> CtrlData(const P &params)
    : WheelData(params.geom), params_(params.ctrl) {
        state_.dAngGearSteerRad = params_.dWheelNeutralPos;
        updateState(WheelState());
        setTarget(PlatformState());
        m_dAngGearSteerTargetRad = params_.dWheelNeutralPos;
    }
};

class UndercarriageDirectCtrl : public UndercarriageCtrlBase < CtrlData > {
public:
    typedef WheelCtrlParams WheelParams;
    UndercarriageDirectCtrl(const std::vector<WheelParams> &params) : UndercarriageCtrlBase(params) {}
};

struct PosCtrlParams {
    /** ------- Position Controller Steer Wheels -------
    * Impedance-Ctrlr Prms
    *  -> model Stiffness via Spring-Damper-Modell
    *  -> only oriented at impedance-ctrl (no forces commanded)
    *  m_dSpring   Spring-constant (elasticity)
    *  m_dDamp             Damping coefficient (also prop. for Velocity Feedforward)
    *  m_dVirtM    Virtual Mass of Spring-Damper System
    *  m_dDPhiMax  maximum angular velocity (cut-off)
    *  m_dDDPhiMax maximum angular acceleration (cut-off)
    */
    double dSpring, dDamp, dVirtM, dDPhiMax, dDDPhiMax;
};

struct WheelCtrlPosParams {
    WheelGeom geom;
    CtrlParams ctrl;
    PosCtrlParams pos_ctrl;
};

struct PosCtrlData : public CtrlData {
    PosCtrlParams pos_params_;

    // previous Commanded deltaPhi e(k-1)
    // double m_dCtrlDeltaPhi; not used
    // previous Commanded Velocity u(k-1)
    double m_dCtrlVelCmdInt;

    virtual void calcControlStep(WheelCommand &command, double dCmdRateS, bool reset);

    virtual void reset();

    PosCtrlData(const WheelCtrlPosParams &params)
    : CtrlData(params), pos_params_(params.pos_ctrl),m_dCtrlVelCmdInt(0) {
        state_.dAngGearSteerRad = params_.dWheelNeutralPos;
        updateState(WheelState());
        setTarget(PlatformState());
        m_dAngGearSteerTargetRad = params_.dWheelNeutralPos;
    }
};

class UndercarriageCtrl : public UndercarriageCtrlBase < PosCtrlData > {
public:
    typedef WheelCtrlPosParams WheelParams;
    UndercarriageCtrl(const std::vector<WheelParams> &params) : UndercarriageCtrlBase(params) {}
};

#endif
