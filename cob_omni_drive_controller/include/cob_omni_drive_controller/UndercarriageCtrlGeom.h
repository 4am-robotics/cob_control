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

class UndercarriageGeomBase
{
public:
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
    // Get result of direct kinematics
    virtual void calcDirect(PlatformState &state) const = 0;

    // Set actual values of wheels (steer/drive velocity/position) (Istwerte)
    virtual void updateWheelStates(const std::vector<WheelState> &states) = 0;

    virtual ~UndercarriageGeomBase() {}

protected:
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
    template<typename V> static void updateWheelStates(V& wheels, const std::vector<WheelState> &states)
    {
        if(wheels.size() != states.size()) throw std::length_error("number of states does not match number of wheels");

        for(size_t i = 0; i < wheels.size(); ++i){
            wheels[i].updateState(states[i]);
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
            const WheelData &wheel = wheels[i];
            const WheelData &other_wheel = wheels[(i+1) % wheels.size()];

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
    std::vector<WheelData> wheels_;
};

class UndercarriageCtrl : public UndercarriageGeomBase {
public:
    struct CtrlParams{
        double dWheelNeutralPos;

        double dMaxDriveRateRadpS;
        double dMaxSteerRateRadpS;

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

    struct WheelParams {
        WheelGeom geom;
        CtrlParams ctrl;
    };

    // Constructor
    UndercarriageCtrl(const std::vector<WheelParams> &params);

    // Get result of direct kinematics
    virtual void calcDirect(PlatformState &state) const;

    // Set actual values of wheels (steer/drive velocity/position) (Istwerte)
    virtual void updateWheelStates(const std::vector<WheelState> &states);

    // Set desired value for Plattform Velocity to UndercarriageCtrl (Sollwertvorgabe)
    void setTarget(const PlatformState &state);

    // Get set point values for the Wheels (including controller) from UndercarriangeCtrl
    void calcControlStep(std::vector<WheelCommand> &commands, double dCmdRateS, bool reset);

    void reset();

    static double limitValue(double value, double limit);

private:
    struct CtrlData : public WheelData {
        CtrlParams params_;

        double m_dAngGearSteerTargetRad; // choosen alternativ for steering angle
        double m_dVelGearDriveTargetRadS;

        // previous Commanded deltaPhi e(k-1)
        // double m_dCtrlDeltaPhi; not used
        // previous Commanded Velocity u(k-1)
        double m_dCtrlVelCmdInt;

        // calculate inverse kinematics
        void setTarget(const PlatformState &state);

        void calcControlStep(WheelCommand &command, double dCmdRateS, bool reset);

        void reset();

        CtrlData(const WheelParams &params)
        : WheelData(params.geom), params_(params.ctrl), /*m_dCtrlDeltaPhi(0),*/  m_dCtrlVelCmdInt(0) {
            state_.dAngGearSteerRad = params_.dWheelNeutralPos;
            updateState(WheelState());
            setTarget(PlatformState());
            m_dAngGearSteerTargetRad = params_.dWheelNeutralPos;
        }
    };

    std::vector<CtrlData> wheels_;
};

#endif
